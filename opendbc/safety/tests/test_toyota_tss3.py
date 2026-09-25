#!/usr/bin/env python3
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.toyota.carcontroller import get_safety_CP
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, EPS_SCALE, CarControllerParams, ToyotaSafetyFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

TRAILER = bytes(4)
DBC = "toyota_tss3_pt_generated"
PACKER = CANPacker(DBC)


def build_signer_requests(seq: int, request: bytes):
  return toyotacan.create_tss3_signer_requests(PACKER, 0, seq, request)


def build_host_application(lat_active: bool = False, angle_raw: int = 0, accel: float = 0.0, request_sequence: int = 0,
                           stock: bytes | None = None) -> bytes:
  stock_values = None
  if stock is not None:
    parser = CANParser(DBC, [("CONTROL_REQUEST", float("nan"))], 2)
    parser.update([(0, [(0x08A, stock[:28] + TRAILER, 2)])])
    stock_values = dict(parser.vl["CONTROL_REQUEST"])
  values = toyotacan.create_tss3_control_request_values(stock_values, lat_active, angle_raw, True, accel, 0.0, request_sequence)
  return PACKER.make_can_msg("CONTROL_REQUEST", 0, values)[1][:28]


def fix_toyota_checksum(msg):
  address, data, bus = msg
  payload = bytearray(data)
  payload[-1] = (address + (address >> 8) + len(payload) + sum(payload[:-1])) & 0xFF
  return address, bytes(payload), bus


class Tss3SafetyHelpers:
  signer_seq = 0

  @staticmethod
  def _admin_msg(arm: bool):
    return libsafety_py.make_CANPacket(0x777, 1, bytes((7, 0xC9, 0xA8, int(arm), 0, 0, 0, 0)))

  @staticmethod
  def _control_request_msg(request: bytes, fd: bool = True):
    msg = libsafety_py.make_CANPacket(0x08A, 0, request + TRAILER)
    msg[0].fd = fd
    return msg

  def _request(self, request: bytes) -> bool:
    """Send the unsigned request to the signer, returns whether the final fragment was allowed."""
    self.signer_seq = self.signer_seq % 0xFF + 1
    ok = False
    for address, dat, bus in build_signer_requests(self.signer_seq, request):
      ok = self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, dat))
    return ok

  def _publish(self, request: bytes) -> bool:
    return self.safety.safety_tx_hook(self._control_request_msg(request))


class TestToyotaTss3CamrySafety(Tss3SafetyHelpers, common.CarSafetyTest, common.AngleSteeringSafetyTest,
                                common.LongitudinalAccelSafetyTest):
  TX_MSGS = [[0x777, 1], [0x777, 0], [0x08A, 0], [0x101, 2], [0x412, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x08A, 0x412)}
  FWD_BLACKLISTED_ADDRS = {2: [0x08A, 0x412]}

  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  STEER_ANGLE_MAX = 1745 * 1024 / 17870
  DEG_TO_CAN = 17870 / 1024
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None
  LATERAL_FREQUENCY = 100

  def setUp(self):
    self.packer = CANPackerSafety("toyota_tss3_pt_generated")
    self.safety = libsafety_py.libsafety
    param = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.TSS3
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._admin_msg(True)))
    self.angle_cmd_count = 0

    fingerprint = {bus: {} for bus in range(8)}
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint, [], True, False, False)
    self.VM = VehicleModel(get_safety_CP())
    self.params = CarControllerParams(self.CP)
    self.params.STEER_STEP = 1 / (0.01 * self.LATERAL_FREQUENCY)

  def _tx(self, msg):
    # like the transport: a CONTROL_REQUEST is first approved by sending it to the signer, then published
    if msg[0].addr != 0x08A:
      return super()._tx(msg)

    request = bytes(msg[0].data)[:28]
    ok = self._request(request) and super()._tx(msg)
    if not ok:
      # re-arm after a rejected frame hands 0x08A back to the FRC
      super()._tx(self._admin_msg(True))
    return ok

  def _application(self, *, angle_raw: int = 0, lat_active: bool = False, accel: float = 0.0) -> bytes:
    return build_host_application(lat_active=lat_active, angle_raw=angle_raw, accel=accel)

  def _application_msg(self, *, angle: float = 0.0, lat_active: bool = False, accel: float = 0.0):
    return self._control_request_msg(self._application(angle_raw=round(angle * self.DEG_TO_CAN), lat_active=lat_active, accel=accel))

  def _accel_msg(self, accel: float):
    return self._application_msg(accel=accel)

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.angle_cmd_count * int(1e6 / self.LATERAL_FREQUENCY))
      self.angle_cmd_count += 1
    return self._application_msg(angle=angle, lat_active=enabled)

  def _angle_raw_cmd_msg(self, angle_raw: int):
    self.safety.set_timer(self.angle_cmd_count * int(1e6 / self.LATERAL_FREQUENCY))
    self.angle_cmd_count += 1
    return self._control_request_msg(self._application(angle_raw=angle_raw, lat_active=True))

  def _angle_meas_msg(self, angle: float):
    coarse = round(angle / 1.5)
    fraction = angle - coarse * 1.5
    values = {"STEER_ANGLE": coarse * 1.5, "STEER_FRACTION": fraction}
    return self.packer.make_can_msg_safety("STEER_ANGLE_SENSOR", 0, values)

  def _get_steer_cmd_angle_max(self, speed):
    return min(get_max_angle_vm(max(speed - 1., 1.), self.VM, self.params), 32767 / self.DEG_TO_CAN)

  def _max_delta_raw(self, speed):
    return min(int(get_max_angle_delta_vm(speed, self.VM, self.params) * self.DEG_TO_CAN) + 1, 1745)

  def test_angle_cmd_when_enabled(self):
    # covered by test_lateral_accel_limit
    pass

  def test_lateral_accel_limit(self):
    for speed in (1., 5., 10., 15., 25., 40.):
      self._reset_speed_measurement(speed + 1.)
      max_angle_raw = min(int(get_max_angle_vm(speed, self.VM, self.params) * self.DEG_TO_CAN) + 1, 1745)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self.safety.set_desired_angle_last(sign * max_angle_raw)
        self.assertTrue(self._tx(self._angle_raw_cmd_msg(sign * max_angle_raw)))

        self.safety.set_controls_allowed(True)
        self.safety.set_desired_angle_last(sign * (max_angle_raw + 1))
        self.assertFalse(self._tx(self._angle_raw_cmd_msg(sign * (max_angle_raw + 1))))

  def test_lateral_jerk_limit(self):
    for speed in (1., 5., 10., 15., 25., 40.):
      self._reset_speed_measurement(speed + 1.)
      max_delta_raw = self._max_delta_raw(speed)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self.safety.set_desired_angle_last(0)
        self.assertTrue(self._tx(self._angle_raw_cmd_msg(sign * max_delta_raw)))

        self.safety.set_controls_allowed(True)
        self.safety.set_desired_angle_last(0)
        self.assertFalse(self._tx(self._angle_raw_cmd_msg(sign * (max_delta_raw + 1))))

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 71.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_requests_are_checked_not_publications(self):
    # a lost signer response skips a generation, the published frames are still approved requests
    self._reset_speed_measurement(10.)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    delta = self._max_delta_raw(9.)
    generations = [self._application(angle_raw=i * delta, lat_active=True) for i in range(3)]
    for i, request in enumerate(generations):
      self.safety.set_timer(i * 10_000)
      self.assertTrue(self._request(request))

    self.assertTrue(self._publish(generations[0]))
    self.assertTrue(self._publish(generations[2]))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

  def test_approved_requests_publish_after_controls_revoked(self):
    self.safety.set_controls_allowed(True)
    request = self._application(angle_raw=10, lat_active=True, accel=1.0)
    self.assertTrue(self._request(request))

    # frames already with the signer are still published, new requests need controls
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._publish(request))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    self.assertFalse(self._request(self._application(angle_raw=10, lat_active=True)))
    self.assertFalse(self._request(self._application(accel=1.0)))
    self.assertTrue(self._request(self._application()))

  def test_each_approval_publishes_once(self):
    request = self._application()
    self.assertTrue(self._request(request))
    self.assertTrue(self._publish(request))
    self.assertFalse(self._publish(request))

  def test_approval_expires(self):
    request = self._application()
    self.assertTrue(self._request(request))
    self.safety.set_timer(100_001)
    self.assertFalse(self._publish(request))

  def test_unapproved_publication_yields_to_frc(self):
    request = self._application()
    self.assertTrue(self._request(request))

    for index in range(28):
      data = bytearray(request)
      data[index] ^= 1
      self.assertFalse(self._publish(bytes(data)), index)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)
      self.assertTrue(self._tx(self._admin_msg(True)))

    self.assertFalse(self.safety.safety_tx_hook(self._control_request_msg(request, fd=False)))
    self.assertTrue(self._tx(self._admin_msg(True)))
    self.assertTrue(self._publish(request))

  def test_request_schema(self):
    request = self._application()
    for index in (0, 1, 2, 3, 4, 5, 6, 7, 13, 14, 15, 16, 17, 20, 22, 23, 25, 27):
      data = bytearray(request)
      data[index] ^= 1
      self.assertFalse(self._request(bytes(data)), index)
    self.assertTrue(self._request(request))

  def test_request_fragments(self):
    request = self._application()
    frames = [libsafety_py.make_CANPacket(address, bus, dat) for address, dat, bus in build_signer_requests(0x5A, request)]

    # out of order or repeated fragments are blocked
    for order in ((1, 2, 3), (0, 2, 3), (0, 1, 3), (0, 1, 2, 2), (0, 1, 1)):
      results = [self.safety.safety_tx_hook(frames[i]) for i in order]
      self.assertFalse(results[-1], order)

    # fragments 2 and 3 must repeat the sequence nibbles of fragments 0 and 1
    for index in (2, 3):
      bad = [bytearray(bytes(f[0].data)[:8]) for f in frames]
      bad[index][0] ^= 1
      results = [self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x777, 0, bytes(d))) for d in bad]
      self.assertFalse(results[index])

    for data in (bytes((0x7F, 1, 2, 3, 4, 5, 6, 7)), bytes((0xC0, 1, 2, 3, 4, 5, 6, 7))):
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x777, 0, data)))

    self.assertTrue(all(self.safety.safety_tx_hook(f) for f in frames))

  def test_admin_msgs(self):
    for action in (False, True):
      self.assertTrue(self._tx(self._admin_msg(action)))
    for index in (0, 1, 2, 4, 5, 6, 7):
      data = bytearray(bytes(self._admin_msg(True)[0].data)[:8])
      data[index] ^= 1
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x777, 1, bytes(data))))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x777, 1, bytes((7, 0xC9, 0xA8, 2, 0, 0, 0, 0)))))

  def test_gas_pressed_does_not_block_accel(self):
    # 0x08A must never stop, the brake ECU arbitrates driver gas
    self.safety.set_controls_allowed(True)
    self.safety.set_gas_pressed_prev(True)
    self.assertTrue(self._tx(self._application_msg(accel=self.MAX_ACCEL)))
    self.assertTrue(self._tx(self._application_msg(accel=self.MIN_ACCEL)))
    self.assertFalse(self._tx(self._application_msg(accel=self.MAX_ACCEL + 0.001)))
    self.assertFalse(self._tx(self._application_msg(accel=self.MIN_ACCEL - 0.001)))

  def test_watchdog_and_release(self):
    self.assertTrue(self._tx(self._application_msg()))
    self.safety.set_timer(99_999)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    self.safety.set_timer(100_001)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

    self.assertTrue(self._tx(self._admin_msg(True)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)
    self.assertTrue(self._tx(self._admin_msg(False)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

  def test_brake_cancel(self):
    _, cancel_data, _ = fix_toyota_checksum((0x101, bytes((0x88, 0, 0, 0, 0, 0, 0, 0)), 2))
    self.assertTrue(self._tx(libsafety_py.make_CANPacket(0x101, 2, cancel_data)))

    brake_off = bytearray(cancel_data)
    brake_off[0] &= ~0x08
    _, brake_off, _ = fix_toyota_checksum((0x101, bytes(brake_off), 2))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x101, 2, brake_off)))

    bad_checksum = bytearray(cancel_data)
    bad_checksum[-1] ^= 1
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x101, 2, bytes(bad_checksum))))

  def test_brake_cancel_from_recorded_state(self):
    # BRAKE_MODULE frames recorded on a Camry, released and pressed
    for frame in ("800000000000008a", "801200000000009c", "800000020000008c", "8800000400000096"):
      with self.subTest(frame=frame):
        parser = CANParser(DBC, [("BRAKE_MODULE", float("nan"))], 0)
        parser.update([(0, [(0x101, bytes.fromhex(frame), 0)])])
        address, dat, bus = toyotacan.create_tss3_brake_cancel_command(PACKER, dict(parser.vl["BRAKE_MODULE"]), 2)
        self.assertTrue(self._tx(libsafety_py.make_CANPacket(address, bus, dat)))

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 0, {"BRAKE_PRESSED": brake}, fix_toyota_checksum)

  def _speed_msg(self, speed):
    values = {f"WHEEL_SPEED_{wheel}": speed * 3.6 for wheel in ("FR", "FL", "RR", "RL")}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values)

  def _speed_msg_2(self, speed):
    return None

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("GAS_PEDAL", 0, {"GAS_PEDAL_USER": gas})

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("CONTROL_REQUEST", 2, {"CRUISE_OPERATING_LATCH": enable})


class TestToyotaTss3CamryStockLongitudinalSafety(Tss3SafetyHelpers, unittest.TestCase):
  STOCK_08A = bytes.fromhex("0000000880002d47fe462afe467fff007fffff35c000100064003c005db7797f")

  def setUp(self):
    self.safety = libsafety_py.libsafety
    param = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.TSS3 | ToyotaSafetyFlags.STOCK_LONGITUDINAL
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, param), 0)
    self.safety.init_tests()
    self.safety.set_timer(0)

  def _rx_stock(self, stock: bytes):
    msg = libsafety_py.make_CANPacket(0x08A, 2, stock)
    msg[0].fd = 1
    self.assertTrue(self.safety.safety_rx_hook(msg))

  def _host_request(self, stock: bytes | None = None, request_sequence: int = 12) -> bytes:
    stock = self.STOCK_08A if stock is None else stock
    return build_host_application(request_sequence=request_sequence, stock=stock)

  def test_arming_requires_recent_frc_request(self):
    self.assertFalse(self.safety.safety_tx_hook(self._admin_msg(True)))
    self._rx_stock(self.STOCK_08A)
    self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))

    self.safety.set_timer(100_001)
    self.assertFalse(self._request(self._host_request()))
    self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(False)))
    self.assertFalse(self.safety.safety_tx_hook(self._admin_msg(True)))

  def test_longitudinal_fields_must_match_frc(self):
    self._rx_stock(self.STOCK_08A)
    self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))

    for index in (3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 17, 20, 22, 23, 27):
      with self.subTest(index=index):
        request = bytearray(self._host_request())
        request[index] ^= 1
        self.assertFalse(self._request(bytes(request)))

    request = self._host_request()
    self.assertTrue(self._request(request))
    self.assertTrue(self._publish(request))

  def test_only_latest_frc_requests_match(self):
    old = bytearray(self.STOCK_08A)
    old[8] ^= 1
    self._rx_stock(bytes(old))
    self._rx_stock(self.STOCK_08A)
    self.assertTrue(self._request(self._host_request(bytes(old))))

    self._rx_stock(self.STOCK_08A)
    self.assertFalse(self._request(self._host_request(bytes(old))))
    self.assertTrue(self._request(self._host_request()))

  def test_one_frc_request_allows_multiple_generations(self):
    self._rx_stock(self.STOCK_08A)
    self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))
    for request_sequence in range(3):
      self.safety.set_timer(request_sequence * 10_000)
      request = self._host_request(request_sequence=request_sequence)
      self.assertTrue(self._request(request))
      self.assertTrue(self._publish(request))


if __name__ == "__main__":
  unittest.main()
