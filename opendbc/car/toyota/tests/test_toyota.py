import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, CanData, structs
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_interface import TSS3_DIST_INVALID, TSS3_GEOMETRY_MSGS, TSS3_SLOTS, RadarInterface
from opendbc.car.toyota.toyotacan import toyota_checksum
from opendbc.car.toyota.tss3 import SIGNER_MAX_PENDING, SIGNER_TIMEOUT_NS, TSS3_AUX_BUS, TSS3_CHASSIS_BUS, TSS3_SOURCE_BUS, \
                                    SignerTransport
from opendbc.car.toyota.values import CAR, DBC, CarControllerParams, ToyotaFlags, ToyotaSafetyFlags, FW_QUERY_CONFIG, \
                                                  PLATFORM_CODE_ECUS, FUZZY_EXCLUDED_PLATFORMS, get_platform_codes
from opendbc.testing import fuzzy_test, parameterized_class

Ecu = CarParams.Ecu


def check_fw_version(fw_version: bytes) -> bool:
  # TODO: just use the FW patterns, need to support all chunks
  return b'?' not in fw_version and b'!' not in fw_version


def cars_with(flags):
  return {c for c in CAR if c.config.flags & flags}


TSS3_PLATFORMS = sorted(cars_with(ToyotaFlags.TSS3))
TSS3_PT_DBC = "toyota_tss3_pt_generated"
TSS3_RADAR_DBC = "toyota_tss3_radar_generated"
MS = 1_000_000

# One frame of each message openpilot reads on TSS3, captured on a Camry Hybrid 2026 with stock ACC engaged
TSS3_MSGS = {
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
  0x27C: bytes.fromhex("91db10c040004000"),
  0x2E5: bytes.fromhex("0000154000000000"),
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
TSS3_FRC_MSGS = {0x08A, 0x251, 0x27C, 0x3F6, 0x412, 0x5AE}  # from the FRC on TSS3_SOURCE_BUS

SIGNER_ARM, SIGNER_RELEASE = bytes.fromhex("07c9a80100000000"), bytes.fromhex("07c9a80000000000")
SIGNER_TRAILER = {"MSG_CNT_LOWER": 1, "RESET_FLAG": 2, "AUTHENTICATOR": 0x1234567}


def tss3_bus(address):
  return TSS3_SOURCE_BUS if address in TSS3_FRC_MSGS else TSS3_CHASSIS_BUS


def tss3_fingerprint():
  fp = {i: {} for i in range(8)}
  for address, data in TSS3_MSGS.items():
    fp[tss3_bus(address)][address] = len(data)
  return fp


def tss3_control(angle=0.0, active=True, accel=0.0, long_active=False, enabled=True, cancel=False, steer_alert=False):
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


def signer_state(responses=(), stock=None, can_valid=True, request_rejected=False, control_request_rejected=False):
  return SimpleNamespace(out=SimpleNamespace(canValid=can_valid, vCruise=70.0), tss3_signer_responses=list(responses),
                         tss3_stock_control_request=stock, tss3_signer_request_rejected=request_rejected,
                         tss3_control_request_rejected=control_request_rejected)


def signer_response(seq, status=0):
  return {"SERVICE_ID": 0xC9, "SIGNER_SEQUENCE": seq, "STATUS": status, "SIGNER_SEQUENCE_INVERTED": seq ^ 0xFF, **SIGNER_TRAILER}


def signer_requests(sends):
  return [(dat, bus) for addr, dat, bus in sends if addr == 0x777 and dat[0] >= 0x80]


def published_requests(sends):
  return [dat for addr, dat, _ in sends if addr == 0x08A]


class TestToyotaInterfaces(unittest.TestCase):
  def test_car_flags(self):
    # Angle and radar-ACC cars are always TSS2 cars
    assert not (cars_with(ToyotaFlags.ANGLE_CONTROL | ToyotaFlags.RADAR_ACC) - cars_with(ToyotaFlags.TSS2))

  def test_lta_platforms(self):
    # At this time, only RAV4 2023 is expected to use LTA/angle control
    assert cars_with(ToyotaFlags.ANGLE_CONTROL) == {CAR.TOYOTA_RAV4_TSS2_2023}

  def test_tss2_dbc(self):
    # We make some assumptions about TSS2 platforms,
    # like looking up certain signals only in this DBC
    for car_model, dbc in DBC.items():
      if car_model.config.flags & ToyotaFlags.TSS2 and not (car_model.config.flags & ToyotaFlags.SECOC):
        assert dbc[Bus.pt] == "toyota_nodsu_pt_generated"

  def test_tss3_dbc(self):
    # TSS3 has its own CarState, CarController, and radar paths, which only read the TSS3 DBCs
    for car_model in TSS3_PLATFORMS:
      with self.subTest(car_model=car_model.value):
        assert DBC[car_model] == {Bus.pt: TSS3_PT_DBC, Bus.radar: TSS3_RADAR_DBC}
        assert not car_model.config.flags & (ToyotaFlags.TSS2 | ToyotaFlags.SECOC | ToyotaFlags.NO_DSU)

  def test_checksum_truncated_frame(self):
    # a frame ending before the bytes the CHECKSUM covers must be rejected, not raise
    parser = CANParser(TSS3_PT_DBC, [("EPS_STATUS", float("nan"))], TSS3_CHASSIS_BUS)
    self.assertEqual(parser.update([(0, [(0x030, TSS3_MSGS[0x030][:6], TSS3_CHASSIS_BUS)])]), set())
    self.assertEqual(parser.update([(0, [(0x030, TSS3_MSGS[0x030], TSS3_CHASSIS_BUS)])]), {0x030})

  def test_essential_ecus(self):
    # Asserts standard ECUs exist for each platform
    common_ecus = {Ecu.fwdRadar, Ecu.fwdCamera}
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        present_ecus = {ecu[0] for ecu in ecus}
        missing_ecus = common_ecus - present_ecus
        assert len(missing_ecus) == 0

        # Some exceptions for other common ECUs
        if car_model not in (CAR.TOYOTA_ALPHARD_TSS2,):
          assert Ecu.abs in present_ecus

        if car_model not in (CAR.TOYOTA_MIRAI,):
          assert Ecu.engine in present_ecus

        if car_model not in (CAR.TOYOTA_PRIUS_V, CAR.LEXUS_CTH):
          assert Ecu.eps in present_ecus


class TestToyotaFingerprint(unittest.TestCase):
  def test_non_essential_ecus(self):
    # Ensures only the cars that have multiple engine ECUs are in the engine non-essential ECU list
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        engine_ecus = {ecu for ecu in ecus if ecu[0] == Ecu.engine}
        assert (len(engine_ecus) > 1) == (car_model in FW_QUERY_CONFIG.non_essential_ecus[Ecu.engine]), \
          f"Car model unexpectedly {'not ' if len(engine_ecus) > 1 else ''}in non-essential list"

  def test_valid_fw_versions(self):
    # Asserts all FW versions are valid
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for fws in ecus.values():
          for fw in fws:
            assert check_fw_version(fw), fw

  # Tests for part numbers, platform codes, and sub-versions which Toyota will use to fuzzy
  # fingerprint in the absence of full FW matches:
  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_platform_code_ecus_available(self):
    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu == Ecu.eps and car_model in (CAR.TOYOTA_PRIUS_V, CAR.LEXUS_CTH,):
            continue
          if platform_code_ecu == Ecu.abs and car_model in (CAR.TOYOTA_ALPHARD_TSS2,):
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU sub-versions

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          codes = dict()
          for fw in fws:
            result = get_platform_codes([fw])
            # Check only one platform code and sub-version
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            assert 1 == len(list(result.values())[0]), f"Unable to parse FW: {fw}"
            codes |= result

          # Toyota places the ECU part number in their FW versions, assert all parsable
          # Note that there is only one unique part number per ECU across the fleet, so this
          # is not important for identification, just a sanity check.
          assert all(code.count(b"-") > 1 for code in codes), f"FW does not have part number: {fw} {codes}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([
      b"F152607140\x00\x00\x00\x00\x00\x00",
      b"F152607171\x00\x00\x00\x00\x00\x00",
      b"F152607110\x00\x00\x00\x00\x00\x00",
      b"F152607180\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {b"F1526-07-1": {b"10", b"40", b"71", b"80"}}

    results = get_platform_codes([
      b"\x028646F4104100\x00\x00\x00\x008646G5301200\x00\x00\x00\x00",
      b"\x028646F4104100\x00\x00\x00\x008646G3304000\x00\x00\x00\x00",
    ])
    assert results == {b"8646F-41-04": {b"100"}}

    # Short version has no part number
    results = get_platform_codes([
      b"\x0235870000\x00\x00\x00\x00\x00\x00\x00\x00A0202000\x00\x00\x00\x00\x00\x00\x00\x00",
      b"\x0235883000\x00\x00\x00\x00\x00\x00\x00\x00A0202000\x00\x00\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {b"58-70": {b"000"}, b"58-83": {b"000"}}

    results = get_platform_codes([
      b"F152607110\x00\x00\x00\x00\x00\x00",
      b"F152607140\x00\x00\x00\x00\x00\x00",
      b"\x028646F4104100\x00\x00\x00\x008646G5301200\x00\x00\x00\x00",
      b"\x0235879000\x00\x00\x00\x00\x00\x00\x00\x00A4701000\x00\x00\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {b"F1526-07-1": {b"10", b"40"}, b"8646F-41-04": {b"100"}, b"58-79": {b"000"}}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      CP = CarParams(carFw=car_fw)
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        # If a platform has multiple matches, add it and its matches
        platforms_with_shared_codes |= {str(platform), *matches}

    assert platforms_with_shared_codes == FUZZY_EXCLUDED_PLATFORMS, (len(platforms_with_shared_codes), len(FW_VERSIONS))


@parameterized_class('car_model', [(c,) for c in TSS3_PLATFORMS])
class TestToyotaTSS3CarInterface(unittest.TestCase):
  car_model: CAR

  @classmethod
  def setUpClass(cls):
    if 'car_model' not in cls.__dict__:
      raise unittest.SkipTest('Base class')

  def setUp(self):
    self.can_t = 1_000 * MS
    self.t = 2_000 * MS
    self.ci = self.interface()

  def params(self, alpha_long=True, is_release=False):
    return CarInterface.get_params(self.car_model, tss3_fingerprint(), [], alpha_long, is_release, False)

  def interface(self, alpha_long=True):
    return CarInterface(self.params(alpha_long))

  def update(self, ci=None, iterations=20, speed_ms=0.0, extra=(), **frames):
    ci = ci or self.ci
    msgs = TSS3_MSGS | {int(k[1:], 16): v for k, v in frames.items()}  # e.g. x08A=b"..."
    msgs[0x0AA] = (6767 + round(speed_ms * 360)).to_bytes(2, "big") * 4
    for _ in range(iterations):
      packets = [CanData(address, data, tss3_bus(address)) for address, data in msgs.items()] + list(extra)
      state = ci.update([(self.can_t, packets)])
      self.can_t += 10 * MS
    return state

  def apply(self, cc):
    self.t += 10 * MS
    return self.ci.apply(cc, self.t)

  def last_request(self):
    return next(reversed(self.ci.CC.signer.pending.values())).values

  def test_params(self):
    for alpha_long in (True, False):
      CP = self.params(alpha_long)
      self.assertEqual((CP.openpilotLongitudinalControl, CP.autoResumeSng), (alpha_long, alpha_long))
      self.assertEqual(bool(CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.STOCK_LONGITUDINAL), not alpha_long)
      self.assertTrue(CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.TSS3)
      self.assertTrue(CP.flags & ToyotaFlags.HAS_BSM)
    self.assertTrue(self.params(alpha_long=False, is_release=True).dashcamOnly)

  def test_carstate(self):
    self.assertEqual((self.ci.can_parsers[Bus.pt].bus, self.ci.can_parsers[Bus.cam].bus), (TSS3_CHASSIS_BUS, TSS3_SOURCE_BUS))
    state = self.update()
    self.assertTrue(state.canValid)
    self.assertEqual(state.gearShifter, structs.CarState.GearShifter.drive)
    self.assertTrue(state.cruiseState.available and state.cruiseState.enabled)
    self.assertFalse(state.carNotReady or state.steerFaultTemporary or state.steerFaultPermanent)

  def test_seatbelt(self):
    for payload, unlatched in (("0000154000000000", False), ("0000554000000000", True), ("0000154000000000", False)):
      with self.subTest(unlatched=unlatched):
        state = self.update(x2E5=bytes.fromhex(payload))
        self.assertTrue(state.canValid)
        self.assertEqual(state.seatbeltUnlatched, unlatched)

  def test_delayed_hold_standstill(self):
    for alpha_long in (False, True):
      ci = self.interface(alpha_long)
      for bytes_4_7, speed, hold in ((b"\x80\x00\x2d\x47", 0, False), (b"\xa0\x00\x2d\x67", 0, True),
                                     (b"\xa0\x00\x2c\x66", 0, True), (b"\x80\x00\x47\x65", 5, False)):
        with self.subTest(alpha_long=alpha_long, request=bytes_4_7.hex()):
          request = TSS3_MSGS[0x08A][:4] + bytes_4_7 + TSS3_MSGS[0x08A][8:]
          self.assertEqual(self.update(ci, speed_ms=speed, x08A=request).cruiseState.standstill, hold and not alpha_long)

  def test_request_loss_fault(self):
    loss = bytes.fromhex("0000001800810b2d000000140000000001210081ff000c63ff000000b0fb684e")
    # pulses on disengagement
    self.assertFalse(self.update(iterations=10, x081=loss).accFaulted)
    self.assertFalse(self.update().accFaulted)

    state = self.update(iterations=100, x081=loss)
    self.assertTrue(state.canValid and state.accFaulted)
    self.assertTrue(self.update().accFaulted)

  def test_stock_pcs(self):
    # FRC CONTROL_REQUESTs from a PCS event: stock ACC braking at -4 m/s^2, both PCS braking request IDs, then stock ACC again
    for request, aeb in (("0000000880002d47f0605ef0607fff007fff000c4000100000000a00fc472d50", False),
                         ("0000000cc0002d8bf0605ef0607fff007fff000b4000100000000b003a194b8d", True),
                         ("00000008c0002d87f0605ef0607fff007fff000d400030000000130006f38fbe", True),
                         ("0000000880002d47f1285ef1287fff007fff00174000300000001f00117281d9", False)):
      with self.subTest(request=request):
        state = self.update(speed_ms=10.0, x08A=bytes.fromhex(request))
        self.assertTrue(state.canValid)
        self.assertEqual((state.stockAeb, state.stockFcw), (aeb, False))

    warning = bytes.fromhex("240004368d080000000000001003000c11ff8a40000f80000000000000000000")
    state = self.update(x5AE=warning)
    self.assertTrue(state.canValid and state.stockFcw)
    self.assertFalse(state.stockAeb)
    self.assertFalse(self.update().stockFcw)

  def test_stock_pcs_releases_control_request(self):
    pcs = bytes.fromhex("0000000cc0002d8bf0605ef0607fff007fff000b4000100000000b003a194b8d")
    release = (0x777, SIGNER_RELEASE, TSS3_AUX_BUS)
    for alpha_long in (False, True):
      with self.subTest(alpha_long=alpha_long):
        self.ci = self.interface(alpha_long)
        self.update(speed_ms=10.0)
        self.assertTrue(any(address == 0x777 and bus == TSS3_CHASSIS_BUS for address, _, bus in self.apply(tss3_control())[1]))

        # the FRC's request goes to the VMC, openpilot stops signing and publishing its own
        self.ci.CC.signer.active = True
        self.update(speed_ms=10.0, x08A=pcs)
        _, sends = self.apply(tss3_control())
        self.assertTrue(release in sends)
        for _ in range(10):
          self.assertFalse(any(address in (0x777, 0x08A) and bus == TSS3_CHASSIS_BUS for address, _, bus in sends))
          _, sends = self.apply(tss3_control())

        self.update(speed_ms=10.0)
        self.assertTrue(any(address == 0x777 and bus == TSS3_CHASSIS_BUS for address, _, bus in self.apply(tss3_control())[1]))

  def test_eps_status(self):
    # recorded with 1.57 Nm of driver torque, and at startup with angle control inhibited
    override = bytes.fromhex("0f0000001501207d10001000230001d7020d0000002301d2000000005c6941f3")
    initializing = bytes.fromhex("00000000000008400000080000100000000000010000000000000000a44271be")
    for eps, pressed in ((override, True), (initializing, False)):
      state = self.update(speed_ms=5, x030=eps)
      self.assertTrue(state.canValid)
      self.assertEqual(state.steeringPressed, pressed)
      self.assertFalse(state.steerFaultTemporary or state.steerFaultPermanent or state.vehicleSensorsInvalid)

    # byte 6 holds EPS_FAULT_INHIBIT (0x04) and DRIVER_TORQUE_INVALID (0x01), 0xF2 sets only the undecoded bits
    checksum = CANPacker(TSS3_PT_DBC).dbc.name_to_msg["EPS_STATUS"].sigs["CHECKSUM"]
    for status, fault in ((0x04, True), (0, False), (0xF2, False)):
      eps = bytearray(TSS3_MSGS[0x030])
      eps[6] = status
      eps[7] = toyota_checksum(0x030, checksum, eps)
      state = self.update(x030=bytes(eps))
      self.assertTrue(state.canValid)
      self.assertEqual((state.steerFaultTemporary, state.steerFaultPermanent), (fault, False))

    eps = bytearray(TSS3_MSGS[0x030])
    eps[7] ^= 0xFF
    self.assertFalse(self.update(x030=bytes(eps)).canValid)

  def test_signer_state_from_can(self):
    self.update(extra=[CanData(0x7A9, bytes.fromhex("c90500fa12345678"), 0), CanData(0x777, b"\xb5" + bytes(7), 192),
                       CanData(0x08A, TSS3_MSGS[0x08A], 192)], iterations=1)
    CS = self.ci.CS
    self.assertEqual([(r["SIGNER_SEQUENCE"], r["SIGNER_SEQUENCE_INVERTED"], r["AUTHENTICATOR"]) for r in CS.tss3_signer_responses],
                     [(5, 0xFA, 0x2345678)])
    self.assertTrue(CS.tss3_signer_request_rejected and CS.tss3_control_request_rejected)
    self.update(iterations=1)
    self.assertFalse(CS.tss3_signer_responses or CS.tss3_signer_request_rejected or CS.tss3_control_request_rejected)

  def test_angle_limits_follow_panda(self):
    state = self.update(speed_ms=25.0)
    measured = state.steeringAngleDeg + state.steeringAngleOffsetDeg
    with patch.object(self.ci.CC.signer, "request_due", return_value=True):
      # the first request is limited from the measured angle with the vehicle model jerk limit
      output, _ = self.apply(tss3_control(20.0))
      self.assertTrue(0.20 < output.steeringAngleDeg - measured < 0.22)
      for _ in range(10):
        output, _ = self.apply(tss3_control(20.0))
      self.assertGreater(output.steeringAngleDeg, measured + 1.0)

      # panda rejected a request and restarted from the measured angle, the controller follows
      self.ci.CC.signer.request_rejected = True
      output, _ = self.apply(tss3_control(20.0))
    self.assertLess(output.steeringAngleDeg - measured, 0.22)

  def test_angle_reference_held_while_signer_queue_full(self):
    self.update(speed_ms=25.0)
    with patch.object(self.ci.CC.signer, "request_due", return_value=True):
      for _ in range(10):
        sent, _ = self.apply(tss3_control(20.0))

    # panda only moves its reference on signed requests, inactive frames included
    with patch.object(self.ci.CC.signer, "request_due", return_value=False):
      held, _ = self.apply(tss3_control(20.0, active=False))
    self.assertEqual(held.steeringAngleDeg, sent.steeringAngleDeg)

  def test_driver_turn_past_command_range_stops_lateral(self):
    angle_max = CarControllerParams.TSS3_ANGLE_LIMITS.STEER_ANGLE_MAX
    self.update(speed_ms=2.0)
    with patch.object(self.ci.CC.signer, "request_due", return_value=True):
      self.ci.CS.out.steeringTorque = 2.5  # a nudge alone keeps lateral
      self.apply(tss3_control())
      self.assertEqual(self.last_request()["LATERAL_REQUEST_ID"], 11)

      for angle in (-370.0, 288.0):
        self.ci.CS.out.steeringAngleDeg = angle - self.ci.CS.out.steeringAngleOffsetDeg
        output, _ = self.apply(tss3_control())
        self.assertEqual(self.last_request()["LATERAL_REQUEST_ID"], 0)
        self.assertAlmostEqual(output.steeringAngleDeg, angle_max if angle > 0 else -angle_max, delta=1e-3)

      # lateral resumes, rate limited from the last request
      self.ci.CS.out.steeringAngleDeg = angle_max - 1 - self.ci.CS.out.steeringAngleOffsetDeg
      previous = output.steeringAngleDeg
      output, _ = self.apply(tss3_control())
      self.assertEqual(self.last_request()["LATERAL_REQUEST_ID"], 11)
      self.assertLessEqual(abs(output.steeringAngleDeg - previous), CarControllerParams.TSS3_ANGLE_LIMITS.MAX_ANGLE_RATE + 1e-3)

  def test_acceleration(self):
    self.update(speed_ms=5.0)
    for requested, expected in ((1.2, 1.2), (3.0, 2.0), (-4.0, -3.5)):
      output, _ = self.apply(tss3_control(active=False, accel=requested, long_active=True))
      self.assertAlmostEqual(output.accel, expected)
    self.assertEqual(self.apply(tss3_control(active=False, accel=1.0))[0].accel, 0.0)

    stock_long = self.interface(alpha_long=False)
    self.update(stock_long, speed_ms=5.0)
    self.assertEqual(stock_long.apply(tss3_control(active=False, accel=1.0, long_active=True), self.t)[0].accel, 0.0)

  def test_cancel_and_hud(self):
    self.update(x412=bytes.fromhex("1000002200ee9307"))
    _, sends = self.apply(tss3_control(cancel=True))
    assert (0x101, bytes.fromhex("8800000100000093"), TSS3_SOURCE_BUS) in sends
    assert (0x412, bytes.fromhex("1000002200ee9307"), TSS3_CHASSIS_BUS) in sends

    # 5 Hz, and immediately on alert edges
    for _ in range(19):
      self.assertFalse(any(addr == 0x412 for addr, _, _ in self.apply(tss3_control())[1]))
    assert (0x412, bytes.fromhex("1000002200ee9307"), TSS3_CHASSIS_BUS) in self.apply(tss3_control())[1]
    assert (0x412, bytes.fromhex("100c002200ee9307"), TSS3_CHASSIS_BUS) in self.apply(tss3_control(steer_alert=True))[1]
    assert (0x412, bytes.fromhex("1000002200ee9307"), TSS3_CHASSIS_BUS) in self.apply(tss3_control())[1]

  def test_lta_switch_button_events(self):
    self.assertEqual(list(self.update(x412=bytes.fromhex("1200002202ee9307")).buttonEvents), [])
    state = self.update(iterations=1, x412=bytes.fromhex("1000002200ee9307"))
    self.assertEqual([(e.type, e.pressed) for e in state.buttonEvents],
                     [(structs.CarState.ButtonEvent.Type.lkas, True), (structs.CarState.ButtonEvent.Type.lkas, False)])

  def test_gap_button_events(self):
    packer = CANPacker(DBC[self.car_model][Bus.pt])
    for gap, pressed in ((0, False), (1, False), (2, True), (2, False), (0, False), (3, False), (1, True), (7, False)):
      with self.subTest(gap=gap, pressed=pressed):
        _, data, _ = packer.make_can_msg("CRUISE_DISPLAY", TSS3_SOURCE_BUS, {"SET_VEHICLE_INTERVAL_TIME": gap})
        state = self.update(iterations=1, x251=data)
        expected = [(structs.CarState.ButtonEvent.Type.gapAdjustCruise, True),
                    (structs.CarState.ButtonEvent.Type.gapAdjustCruise, False)] if pressed else []
        self.assertEqual([(e.type, e.pressed) for e in state.buttonEvents], expected)


class TestToyotaTSS3SignerTransport(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(TSS3_PT_DBC)
    self.signer = SignerTransport(self.packer, stock_longitudinal=False)
    self.t = 1_000 * MS

  def step(self, CS=None, enabled=True, angle=1.0, accel=0.5):
    CS = CS or signer_state()
    sends = self.signer.receive(CS, enabled, self.t)
    sends += self.signer.send(CS, enabled, True, angle, True, accel, self.t)
    self.t += 10 * MS
    return sends

  def test_request_fragments_and_signed_publication(self):
    sends = self.step()
    fragments = signer_requests(sends)
    self.assertEqual([dat[0] for dat, _ in fragments], [0x81, 0x90, 0xA1, 0xB0])
    self.assertTrue(all(bus == TSS3_CHASSIS_BUS for _, bus in fragments))
    application = b"".join(dat[1:] for dat, _ in fragments)

    sends = self.step(signer_state([signer_response(1)]))
    self.assertEqual(sends[0], (0x777, SIGNER_ARM, TSS3_AUX_BUS))
    frame = published_requests(sends)[0]
    self.assertEqual(frame[:28], application)
    parser = CANParser(TSS3_PT_DBC, [("CONTROL_REQUEST", float("nan"))], TSS3_CHASSIS_BUS)
    parser.update([(0, [(0x08A, frame, TSS3_CHASSIS_BUS)])])
    self.assertEqual({s: parser.vl["CONTROL_REQUEST"][s] for s in SIGNER_TRAILER}, SIGNER_TRAILER)
    self.assertEqual(parser.vl["CONTROL_REQUEST"]["LATERAL_REQUEST_ID"], 11)
    self.assertAlmostEqual(parser.vl["CONTROL_REQUEST"]["LONGITUDINAL_REQUEST_ACCEL_UPPER"], 0.5)

  def test_stock_longitudinal_only_replaces_lateral_fields_and_sequence(self):
    stock_frame = TSS3_MSGS[0x08A]
    parser = CANParser(TSS3_PT_DBC, [("CONTROL_REQUEST", float("nan"))], TSS3_SOURCE_BUS)
    parser.update([(0, [(0x08A, stock_frame, TSS3_SOURCE_BUS)])])
    self.signer = SignerTransport(self.packer, stock_longitudinal=True)
    stock = signer_state(stock=dict(parser.vl["CONTROL_REQUEST"]))
    application = b"".join(dat[1:] for dat, _ in signer_requests(self.step(stock)))
    for i, (ours, theirs) in enumerate(zip(application, stock_frame[:28], strict=True)):
      mask = {18: 0, 19: 0, 24: 0, 25: 0, 21: 0xC0, 26: 0xC0}.get(i, 0xFF)  # as panda checks it
      self.assertEqual(ours & mask, theirs & mask, i)

  def test_pipeline_publishes_in_order_at_100hz(self):
    for _ in range(SIGNER_MAX_PENDING + 2):
      self.step()
    self.assertEqual(len(self.signer.pending), SIGNER_MAX_PENDING)

    sequences = []
    for seq in range(1, 9):
      frames = published_requests(self.step(signer_state([signer_response(seq)])))
      self.assertEqual(len(frames), 1)
      sequences.append(frames[0][26] & 0x3F)
    self.assertEqual(sequences, list(range(8)))

  def test_lost_response_drops_earlier_requests_and_keeps_request_timing(self):
    self.step()
    self.step()
    # response for the second request, the first was lost
    self.assertEqual(published_requests(self.step(signer_state([signer_response(2)])))[0][26] & 0x3F, 1)
    self.assertNotIn(1, self.signer.pending)

    self.step(signer_state([signer_response(3)]))
    # request 4 was lost, request 5 is two steps after the last published and waits 20 ms
    self.assertEqual(published_requests(self.step(signer_state([signer_response(5)]))), [])

  def test_signer_error_and_invalid_responses_are_skipped(self):
    self.step()
    bad = dict(signer_response(1), SIGNER_SEQUENCE_INVERTED=0)
    self.assertEqual(published_requests(self.step(signer_state([bad, signer_response(99)]))), [])
    assert 1 in self.signer.pending
    self.step(signer_state([signer_response(1, status=5)]))
    self.assertNotIn(1, self.signer.pending)

  def test_release_on_disable_invalid_can_and_timeout(self):
    for release in ({"enabled": False}, {"CS": signer_state(can_valid=False)}):
      with self.subTest(**{k: str(v) for k, v in release.items()}):
        self.setUp()
        self.step()
        self.step(signer_state([signer_response(1)]))
        sends = self.step(**release)
        assert (0x777, SIGNER_RELEASE, TSS3_AUX_BUS) in sends
        self.assertEqual((self.signer.active, len(self.signer.pending)), (False, 0))

    self.setUp()
    self.step()
    self.step(signer_state([signer_response(1)]))
    for _ in range(SIGNER_TIMEOUT_NS // (10 * MS) + 1):
      sends = self.step()
    assert (0x777, SIGNER_RELEASE, TSS3_AUX_BUS) in sends

  def test_rejected_publication_releases_and_rearms(self):
    self.step()
    self.step()
    self.step(signer_state([signer_response(1)]))
    sends = self.step(signer_state([signer_response(2)], control_request_rejected=True))
    self.assertEqual([dat for addr, dat, _ in sends if addr == 0x777 and dat[0] == 0x07], [SIGNER_RELEASE])
    self.step()
    assert (0x777, SIGNER_ARM, TSS3_AUX_BUS) in self.step(signer_state([signer_response(4)]))

  def test_angle_reference_reset(self):
    self.assertTrue(self.signer.angle_reference_reset(self.t))
    self.step()
    self.assertFalse(self.signer.angle_reference_reset(self.t))
    self.step(signer_state(request_rejected=True))
    self.assertTrue(self.signer.angle_reference_reset(self.t))
    self.assertFalse(self.signer.angle_reference_reset(self.t))
    self.assertTrue(self.signer.angle_reference_reset(self.t + 200 * MS))


@parameterized_class('car_model', [(c,) for c in TSS3_PLATFORMS])
class TestToyotaTSS3RadarInterface(unittest.TestCase):
  car_model: CAR
  BUS = 1
  EMPTY = {'dist': TSS3_DIST_INVALID, 'lat': 0, 'vrel': 0, 'state': 0, 'new': 0, 'ended': 0}
  TRACK = {'dist': 20, 'lat': 1, 'vrel': -1, 'state': 2}

  @classmethod
  def setUpClass(cls):
    if 'car_model' not in cls.__dict__:
      raise unittest.SkipTest('Base class')

  def setUp(self):
    CP = CarInterface.get_non_essential_params(self.car_model)
    CP.radarUnavailable = False
    self.ri = RadarInterface(CP)
    self.packer = CANPacker(DBC[self.car_model][Bus.radar])
    self.t = 1_000 * MS

  def geometry_msg(self, bank, objects=None):
    geo = {}
    for slot in range(TSS3_SLOTS):
      obj = self.EMPTY | (objects or {}).get(bank * TSS3_SLOTS + slot, {})
      geo |= {f'DIST_{slot}': obj['dist'], f'LAT_{slot}': obj['lat']}
    return self.packer.make_can_msg(f'OBJECT_GEOMETRY_{bank}', self.BUS, geo)

  def motion_msg(self, bank, objects=None):
    motion = {}
    for slot in range(TSS3_SLOTS):
      obj = self.EMPTY | (objects or {}).get(bank * TSS3_SLOTS + slot, {})
      motion |= {f'VREL_{slot}': obj['vrel'], f'TRACK_STATE_{slot}': obj['state'],
                 f'NEW_TRACK_{slot}': obj['new'], f'TRACK_ENDED_{slot}': obj['ended']}
    return self.packer.make_can_msg(f'OBJECT_MOTION_{bank}', self.BUS, motion)

  def send(self, msgs):
    rr = self.ri.update([(self.t, list(msgs))])
    self.t += 50 * MS
    return rr

  def update(self, objects=None, geometry=True):
    msgs = []
    for bank in range(len(TSS3_GEOMETRY_MSGS)):
      if geometry:
        msgs.append(self.geometry_msg(bank, objects))
      msgs.append(self.motion_msg(bank, objects))
    return self.send(msgs)

  def track_ids(self, objects=None):
    return [pt.trackId for pt in self.update(objects).points]

  def test_geometry_all_slots(self):
    n = len(TSS3_GEOMETRY_MSGS) * TSS3_SLOTS
    objects = {key: {'dist': 10 + key, 'lat': (key - n // 2) * 0.2, 'vrel': -key * 0.5, 'state': 2, 'new': 1} for key in range(n)}
    rr = self.update(objects)
    self.assertFalse(rr.errors.canError)
    self.assertEqual(len(rr.points), n)
    self.assertEqual(len({pt.trackId for pt in rr.points}), n)
    for key, obj in objects.items():
      pt = self.ri.pts[key]
      self.assertAlmostEqual(pt.dRel, obj['dist'], places=3)
      self.assertAlmostEqual(pt.yRel, obj['lat'], places=3)
      self.assertAlmostEqual(pt.vRel, obj['vrel'], places=3)

  def test_track_lifecycle(self):
    first = self.track_ids({0: self.TRACK | {'new': 1}})
    self.assertEqual(self.track_ids({0: self.TRACK}), first)
    replaced = self.track_ids({0: self.TRACK | {'new': 1}})
    self.assertNotEqual(replaced, first)
    replaced_again = self.track_ids({0: self.TRACK | {'new': 1, 'ended': 1}})
    self.assertEqual(len(replaced_again), 1)
    self.assertNotEqual(replaced_again, replaced)
    self.assertEqual(self.track_ids({0: self.TRACK | {'ended': 1}}), [])
    self.assertNotEqual(self.track_ids({0: self.TRACK}), replaced_again)

  def test_empty_slots(self):
    for empty in ({'state': 0}, {'dist': 0}, {'dist': self.EMPTY['dist']}):
      with self.subTest(empty=empty):
        first = self.track_ids({3: self.TRACK})
        self.assertEqual(self.track_ids({3: self.TRACK | empty}), [])
        self.assertNotEqual(self.track_ids({3: self.TRACK}), first)
    self.assertEqual(self.track_ids(), [])

  def test_can_error_clears_points(self):
    self.assertEqual(len(self.update({0: self.TRACK}).points), 1)
    for _ in range(20):
      rr = self.update({0: self.TRACK}, geometry=False)
    self.assertTrue(rr.errors.canError)
    self.assertEqual(len(rr.points), 0)
    self.assertEqual(self.ri.pts, {})

  def test_missed_geometry(self):
    first = self.track_ids({0: self.TRACK | {'new': 1}})
    rr = self.send([self.geometry_msg(1), self.geometry_msg(2),
                    self.motion_msg(0, {0: self.TRACK | {'vrel': -5}}), self.motion_msg(1), self.motion_msg(2)])
    self.assertFalse(rr.errors.canError)
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 20, -1)])
    rr = self.update({0: self.TRACK | {'dist': 25, 'vrel': -5}})
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 25, -5)])

  def test_missed_motion_after_new_track(self):
    first = self.track_ids({0: self.TRACK | {'new': 1}})
    rr = self.send([self.geometry_msg(0, {0: self.TRACK | {'dist': 25}}), self.geometry_msg(1), self.geometry_msg(2),
                    self.motion_msg(1), self.motion_msg(2)])
    self.assertFalse(rr.errors.canError)
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 20, -1)])
    rr = self.update({0: self.TRACK | {'dist': 25, 'vrel': -5}})
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 25, -5)])

  def test_lifecycle_without_geometry_invalidates(self):
    for event in ({'new': 1}, {'ended': 1}, {'state': 0}):
      with self.subTest(event=event):
        first = self.track_ids({0: self.TRACK})
        rr = self.send([self.geometry_msg(1), self.geometry_msg(2),
                        self.motion_msg(0, {0: self.TRACK | event}), self.motion_msg(1), self.motion_msg(2)])
        self.assertEqual(len(rr.points), 0)
        recovered = self.track_ids({0: self.TRACK})
        self.assertEqual(len(recovered), 1)
        self.assertNotEqual(recovered, first)

  def test_split_updates(self):
    first = self.track_ids({0: self.TRACK | {'new': 1}})
    self.assertIsNone(self.send([self.geometry_msg(0, {0: self.TRACK | {'dist': 25}})]))
    rr = self.send([self.motion_msg(0, {0: self.TRACK | {'vrel': -5}}), self.motion_msg(1), self.motion_msg(2)])
    self.assertEqual([(p.trackId, p.dRel, p.vRel) for p in rr.points], [(first[0], 25, -5)])
