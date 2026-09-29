import unittest
from unittest.mock import patch

from opendbc.car import Bus, CanData, structs
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, CarControllerParams, ToyotaFlags, ToyotaSafetyFlags

CAMRY_COMMON = {
  0x025: bytes.fromhex("000100005000007e0000000000000000000000000000000000000000bb6fee54"),
  0x030: bytes.fromhex("000000ffc400201b00ffc0ff9e00003f22000000ff9e007000000000b96152f6"),
  0x081: bytes.fromhex("00000018fd8a3f670000000400000000002800490049100c00491fbcb0598655"),
  0x08A: bytes.fromhex("0000000880002d47fe462afe467fff007fffff35c000100064003c005db7797f"),
  0x0AA: bytes.fromhex("1a6f1a6f1a6f1a6f"),
  0x0FE: bytes.fromhex("567d393f0000c36200000000000000002640000000ff000000000000d54aaf10"),
  0x101: bytes.fromhex("800000010000008b"),
  0x116: bytes.fromhex("000000007b4b235a"),
  0x127: bytes.fromhex("00100000003e8d0b"),
  0x251: bytes.fromhex("c01015908030a080"),
  0x3B7: bytes.fromhex("0000000020000008"),
  0x3F6: bytes.fromhex("81ea6e0480ba4808"),
  0x412: bytes.fromhex("140c404401ee9307"),
  0x51E: bytes.fromhex("80006e0000000000"),
  0x5AE: bytes.fromhex("240000370b080800000000001003800c19df8a40001000000000000000000000"),
  0x610: bytes.fromhex("00001d4ed0fffc00"),
  0x614: bytes.fromhex("00004a3000003303"),
  0x620: bytes.fromhex("000000008000001a"),
  0x622: bytes.fromhex("0000000000730000"),
}
FRC_IDS = {0x08A, 0x251, 0x3F6, 0x412, 0x5AE}  # from the FRC on bus 2
ANGLE_MAX = CarControllerParams.TSS3_ANGLE_LIMITS.STEER_ANGLE_MAX


def fingerprint():
  fp = {i: {} for i in range(8)}
  for address, data in CAMRY_COMMON.items():
    fp[2 if address in FRC_IDS else 0][address] = len(data)
  return fp


def update_state(ci, iterations=20, speed_ms=0.0, extra=(), **frames):
  msgs = CAMRY_COMMON | {int(k[1:], 16): v for k, v in frames.items()}  # e.g. x08A=b"..."
  msgs[0x0AA] = (6767 + round(speed_ms * 360)).to_bytes(2, "big") * 4
  for _ in range(iterations):
    packets = [CanData(address, data, 2 if address in FRC_IDS else 0) for address, data in msgs.items()] + list(extra)
    state = ci.update([(update_state.t, packets)])
    update_state.t += 10_000_000
  return state
update_state.t = 1_000_000_000


def control(angle=0.0, active=True, accel=0.0, long_active=False, enabled=True, cancel=False, steer_alert=False):
  cc = structs.CarControl()
  cc.enabled = enabled
  cc.latActive = enabled and active
  cc.longActive = enabled and long_active
  cc.cruiseControl.cancel = cancel
  cc.actuators.steeringAngleDeg = angle
  cc.actuators.accel = accel
  cc.hudControl.leftLaneVisible = cc.hudControl.rightLaneVisible = True
  if steer_alert:
    cc.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.steerRequired
  return cc.as_reader()


class TestToyotaCamryTSS3(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], True, False, False)
    self.ci = CarInterface(self.CP)
    self.t = 2_000_000_000

  def apply(self, cc):
    self.t += 10_000_000
    return self.ci.apply(cc, self.t)

  def last_request(self):
    return next(reversed(self.ci.CC.signer.pending.values())).values

  def test_params(self):
    for alpha_long in (True, False):
      CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], alpha_long, False, False)
      self.assertEqual((CP.openpilotLongitudinalControl, CP.autoResumeSng), (alpha_long, alpha_long))
      self.assertEqual(bool(CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL), not alpha_long)
      self.assertTrue(CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.TSS3)
      self.assertTrue(CP.flags & ToyotaFlags.HAS_BSM)
    self.assertTrue(CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], False, True, False).dashcamOnly)

  def test_carstate(self):
    self.assertEqual((self.ci.can_parsers[Bus.pt].bus, self.ci.can_parsers[Bus.cam].bus), (0, 2))
    state = update_state(self.ci)
    self.assertTrue(state.canValid)
    self.assertEqual(state.gearShifter, structs.CarState.GearShifter.drive)
    self.assertTrue(state.cruiseState.available and state.cruiseState.enabled)
    self.assertFalse(state.carNotReady or state.steerFaultTemporary or state.steerFaultPermanent)

  def test_delayed_hold_standstill(self):
    for alpha_long in (False, True):
      ci = CarInterface(CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], alpha_long, False, False))
      for bytes_4_7, speed, hold in ((b"\x80\x00\x2d\x47", 0, False), (b"\xa0\x00\x2d\x67", 0, True),
                                     (b"\xa0\x00\x2c\x66", 0, True), (b"\x80\x00\x47\x65", 5, False)):
        with self.subTest(alpha_long=alpha_long, request=bytes_4_7.hex()):
          request = CAMRY_COMMON[0x08A][:4] + bytes_4_7 + CAMRY_COMMON[0x08A][8:]
          self.assertEqual(update_state(ci, speed_ms=speed, x08A=request).cruiseState.standstill, hold and not alpha_long)

  def test_request_loss_fault(self):
    loss = bytes.fromhex("0000001800810b2d000000140000000001210081ff000c63ff000000b0fb684e")
    # pulses on disengagement
    self.assertFalse(update_state(self.ci, iterations=10, x081=loss).accFaulted)
    self.assertFalse(update_state(self.ci).accFaulted)

    state = update_state(self.ci, iterations=100, x081=loss)
    self.assertTrue(state.canValid and state.accFaulted)
    self.assertTrue(update_state(self.ci).accFaulted)

  def test_stock_pcs(self):
    # FRC CONTROL_REQUESTs from a PCS event: stock ACC braking at -4 m/s^2, both PCS braking request IDs, then stock ACC again
    for request, aeb in (("0000000880002d47f0605ef0607fff007fff000c4000100000000a00fc472d50", False),
                         ("0000000cc0002d8bf0605ef0607fff007fff000b4000100000000b003a194b8d", True),
                         ("00000008c0002d87f0605ef0607fff007fff000d400030000000130006f38fbe", True),
                         ("0000000880002d47f1285ef1287fff007fff00174000300000001f00117281d9", False)):
      with self.subTest(request=request):
        state = update_state(self.ci, speed_ms=10.0, x08A=bytes.fromhex(request))
        self.assertTrue(state.canValid)
        self.assertEqual((state.stockAeb, state.stockFcw), (aeb, False))

    warning = bytes.fromhex("240004368d080000000000001003000c11ff8a40000f80000000000000000000")
    state = update_state(self.ci, x5AE=warning)
    self.assertTrue(state.canValid and state.stockFcw)
    self.assertFalse(state.stockAeb)
    self.assertFalse(update_state(self.ci).stockFcw)

  def test_stock_pcs_releases_control_request(self):
    pcs = bytes.fromhex("0000000cc0002d8bf0605ef0607fff007fff000b4000100000000b003a194b8d")
    release = (0x777, bytes((7, 0xC9, 0xA8, 0, 0, 0, 0, 0)), 1)
    for alpha_long in (False, True):
      with self.subTest(alpha_long=alpha_long):
        self.ci = CarInterface(CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], alpha_long, False, False))
        update_state(self.ci, speed_ms=10.0)
        self.assertTrue(any(address == 0x777 and bus == 0 for address, _, bus in self.apply(control())[1]))

        # the FRC's request goes to the VMC, openpilot stops signing and publishing its own
        self.ci.CC.signer.active = True
        update_state(self.ci, speed_ms=10.0, x08A=pcs)
        _, sends = self.apply(control())
        self.assertTrue(release in sends)
        for _ in range(10):
          self.assertFalse(any(address in (0x777, 0x08A) and bus == 0 for address, _, bus in sends))
          _, sends = self.apply(control())

        update_state(self.ci, speed_ms=10.0)
        self.assertTrue(any(address == 0x777 and bus == 0 for address, _, bus in self.apply(control())[1]))

  def test_eps_status(self):
    override = bytes.fromhex("12000003330930b9130330053c800e99030b0000053c07b50000000042c3b381")
    initializing = bytes.fromhex("00000000170000500000100026820000000000010000ffff00000000b280595f")
    for eps, pressed in ((override, True), (initializing, False)):
      state = update_state(self.ci, speed_ms=5, x030=eps)
      self.assertTrue(state.canValid)
      self.assertEqual(state.steeringPressed, pressed)
      self.assertFalse(state.steerFaultTemporary or state.steerFaultPermanent or state.vehicleSensorsInvalid)

    for status, fault in ((0x04, True), (0, False), (0xF2, False)):
      eps = bytearray(CAMRY_COMMON[0x030])
      eps[6] = status
      eps[7] = (sum(eps[:7]) + 0x38) & 0xFF
      state = update_state(self.ci, x030=bytes(eps))
      self.assertEqual((state.steerFaultTemporary, state.steerFaultPermanent), (fault, False))

  def test_signer_state_from_can(self):
    update_state(self.ci, extra=[CanData(0x7A9, bytes.fromhex("c90500fa12345678"), 0), CanData(0x777, b"\xb5" + bytes(7), 192),
                                 CanData(0x08A, CAMRY_COMMON[0x08A], 192)], iterations=1)
    CS = self.ci.CS
    self.assertEqual([(r["SIGNER_SEQUENCE"], r["SIGNER_SEQUENCE_INVERTED"], r["AUTHENTICATOR"]) for r in CS.tss3_signer_responses],
                     [(5, 0xFA, 0x2345678)])
    self.assertTrue(CS.tss3_signer_request_rejected and CS.tss3_control_request_rejected)
    update_state(self.ci, iterations=1)
    self.assertFalse(CS.tss3_signer_responses or CS.tss3_signer_request_rejected or CS.tss3_control_request_rejected)

  def test_angle_limits_follow_panda(self):
    state = update_state(self.ci, speed_ms=25.0)
    measured = state.steeringAngleDeg + state.steeringAngleOffsetDeg
    with patch.object(self.ci.CC.signer, "request_due", return_value=True):
      # the first request is limited from the measured angle with the vehicle model jerk limit
      output, _ = self.apply(control(20.0))
      self.assertTrue(0.20 < output.steeringAngleDeg - measured < 0.22)
      for _ in range(10):
        output, _ = self.apply(control(20.0))
      self.assertGreater(output.steeringAngleDeg, measured + 1.0)

      # panda rejected a request and restarted from the measured angle, the controller follows
      self.ci.CC.signer.request_rejected = True
      output, _ = self.apply(control(20.0))
    self.assertLess(output.steeringAngleDeg - measured, 0.22)

  def test_angle_reference_held_while_signer_queue_full(self):
    update_state(self.ci, speed_ms=25.0)
    with patch.object(self.ci.CC.signer, "request_due", return_value=True):
      for _ in range(10):
        sent, _ = self.apply(control(20.0))

    # panda only moves its reference on signed requests, inactive frames included
    with patch.object(self.ci.CC.signer, "request_due", return_value=False):
      held, _ = self.apply(control(20.0, active=False))
    self.assertEqual(held.steeringAngleDeg, sent.steeringAngleDeg)

  def test_driver_turn_past_command_range_stops_lateral(self):
    update_state(self.ci, speed_ms=2.0)
    with patch.object(self.ci.CC.signer, "request_due", return_value=True):
      self.ci.CS.out.steeringTorque = 2.5  # a nudge alone keeps lateral
      self.apply(control())
      self.assertEqual(self.last_request()["LATERAL_REQUEST_ID"], 11)

      for angle in (-370.0, 288.0):
        self.ci.CS.out.steeringAngleDeg = angle - self.ci.CS.out.steeringAngleOffsetDeg
        output, _ = self.apply(control())
        self.assertEqual(self.last_request()["LATERAL_REQUEST_ID"], 0)
        self.assertAlmostEqual(output.steeringAngleDeg, ANGLE_MAX if angle > 0 else -ANGLE_MAX, delta=1e-3)

      # lateral resumes, rate limited from the last request
      self.ci.CS.out.steeringAngleDeg = ANGLE_MAX - 1 - self.ci.CS.out.steeringAngleOffsetDeg
      previous = output.steeringAngleDeg
      output, _ = self.apply(control())
      self.assertEqual(self.last_request()["LATERAL_REQUEST_ID"], 11)
      self.assertLessEqual(abs(output.steeringAngleDeg - previous), CarControllerParams.TSS3_ANGLE_LIMITS.MAX_ANGLE_RATE + 1e-3)

  def test_acceleration(self):
    update_state(self.ci, speed_ms=5.0)
    for requested, expected in ((1.2, 1.2), (3.0, 2.0), (-4.0, -3.5)):
      output, _ = self.apply(control(active=False, accel=requested, long_active=True))
      self.assertAlmostEqual(output.accel, expected)
    self.assertEqual(self.apply(control(active=False, accel=1.0))[0].accel, 0.0)

    stock_long = CarInterface(CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint(), [], False, False, False))
    update_state(stock_long, speed_ms=5.0)
    self.assertEqual(stock_long.apply(control(active=False, accel=1.0, long_active=True), self.t)[0].accel, 0.0)

  def test_cancel_and_hud(self):
    update_state(self.ci, x412=bytes.fromhex("1000002200ee9307"))
    _, sends = self.apply(control(cancel=True))
    assert (0x101, bytes.fromhex("8800000100000093"), 2) in sends
    assert (0x412, bytes.fromhex("1400004401ee9307"), 0) in sends

    # 5 Hz, and immediately on alert edges
    for _ in range(19):
      self.assertFalse(any(addr == 0x412 for addr, _, _ in self.apply(control())[1]))
    assert (0x412, bytes.fromhex("1400004401ee9307"), 0) in self.apply(control())[1]
    assert (0x412, bytes.fromhex("140c004401ee9307"), 0) in self.apply(control(steer_alert=True))[1]
    assert (0x412, bytes.fromhex("1400004401ee9307"), 0) in self.apply(control())[1]

  def test_lta_switch_button_events(self):
    self.assertEqual(list(update_state(self.ci, x412=bytes.fromhex("1200002202ee9307")).buttonEvents), [])
    state = update_state(self.ci, iterations=1, x412=bytes.fromhex("1000002200ee9307"))
    self.assertEqual([(e.type, e.pressed) for e in state.buttonEvents],
                     [(structs.CarState.ButtonEvent.Type.lkas, True), (structs.CarState.ButtonEvent.Type.lkas, False)])


if __name__ == "__main__":
  unittest.main()
