#!/usr/bin/env python3
import numpy as np
import random
import unittest
import itertools

from opendbc.can import CANPacker, CANParser
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.carcontroller import get_safety_CP
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, EPS_SCALE, CarControllerParams, ToyotaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common

TOYOTA_COMMON_TX_MSGS = [[0x2E4, 0], [0x191, 0], [0x412, 0], [0x343, 0], [0x1D2, 0]]  # LKAS + LTA + ACC & PCM cancel cmds
TOYOTA_SECOC_TX_MSGS = [[0x131, 0], [0x183, 0]] + TOYOTA_COMMON_TX_MSGS
TOYOTA_COMMON_LONG_TX_MSGS = [[0x283, 0], [0x2E6, 0], [0x2E7, 0], [0x33E, 0], [0x344, 0], [0x365, 0], [0x366, 0], [0x4CB, 0],  # DSU bus 0
                              [0x128, 1], [0x141, 1], [0x160, 1], [0x161, 1], [0x470, 1],  # DSU bus 1
                              [0x411, 0],  # PCS_HUD
                              [0x750, 0]]  # radar diagnostic address

TSS3_DBC = "toyota_tss3_pt_generated"
TSS3_PACKER = CANPacker(TSS3_DBC)
TSS3_TRAILER = bytes.fromhex("5db7797f")  # from a recorded CONTROL_REQUEST


class TestToyotaSafetyBase(common.CarSafetyTest, common.LongitudinalAccelSafetyTest):

  DBC = "toyota_nodsu_pt_generated"
  SAFETY_MODEL = CarParams.SafetyModel.toyota

  TX_MSGS = TOYOTA_COMMON_TX_MSGS + TOYOTA_COMMON_LONG_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x343)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191, 0x343]}
  EPS_SCALE = 73

  def _torque_meas_msg(self, torque: int, driver_torque: int | None = None):
    values = {"STEER_TORQUE_EPS": (torque / self.EPS_SCALE) * 100.}
    if driver_torque is not None:
      values["STEER_TORQUE_DRIVER"] = driver_torque
    return self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)

  # Both torque and angle safety modes test with each other's steering commands
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEER_TORQUE_CMD": torque, "STEER_REQUEST": steer_req}
    return self.packer.make_can_msg_safety("STEERING_LKA", 0, values)

  def _angle_meas_msg(self, angle: float, steer_angle_initializing: bool = False):
    # This creates a steering torque angle message. Not set on all platforms,
    # relative to init angle on some older TSS2 platforms. Only to be used with LTA
    values = {"STEER_ANGLE": angle, "STEER_ANGLE_INITIALIZING": int(steer_angle_initializing)}
    return self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    return self._lta_msg(int(enabled), int(enabled), angle, torque_wind_down=100 if enabled else 0)

  def _lta_msg(self, req, req2, angle_cmd, torque_wind_down=100):
    values = {"STEER_REQUEST": req, "STEER_REQUEST_2": req2, "STEER_ANGLE_CMD": angle_cmd, "TORQUE_WIND_DOWN": torque_wind_down}
    return self.packer.make_can_msg_safety("STEERING_LTA", 0, values)

  def _accel_msg_343(self, accel, cancel_req=0):
    values = {"ACCEL_CMD": accel, "CANCEL_REQ": cancel_req}
    return self.packer.make_can_msg_safety("ACC_CONTROL", 0, values)

  def _accel_msg(self, accel, cancel_req=0):
    return self._accel_msg_343(accel, cancel_req)

  def _speed_msg(self, speed, quality_flag=True):
    values = {("WHEEL_SPEED_%s" % n): speed * 3.6 for n in ["FR", "FL", "RR", "RL"]}
    if not quality_flag:
      values |= {"WHEEL_SPEED_%s_FAULT" % n: 1.0 for n in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 0, values)

  def _user_gas_msg(self, gas):
    cruise_active = self.safety.get_controls_allowed()
    values = {"GAS_RELEASED": not gas, "CRUISE_ACTIVE": cruise_active}
    return self.packer.make_can_msg_safety("PCM_CRUISE", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_ACTIVE": enable}
    return self.packer.make_can_msg_safety("PCM_CRUISE", 0, values)

  def test_diagnostics(self, stock_longitudinal: bool = False, ecu_disabled: bool = True):
    for should_tx, msg in ((False, b"\x6D\x02\x3E\x00\x00\x00\x00\x00"),  # fwdCamera tester present
                           (False, b"\x0F\x03\xAA\xAA\x00\x00\x00\x00"),  # non-tester present
                           (True, b"\x0F\x02\x3E\x00\x00\x00\x00\x00")):
      tester_present = libsafety_py.make_CANPacket(0x750, 0, msg)
      self.assertEqual(should_tx and ecu_disabled and not stock_longitudinal, self._tx(tester_present))

  def test_block_aeb(self, stock_longitudinal: bool = False):
    for controls_allowed in (True, False):
      for bad in (True, False):
        for _ in range(10):
          self.safety.set_controls_allowed(controls_allowed)
          dat = [random.randint(1, 255) for _ in range(7)]
          if not bad:
            dat = [0]*6 + dat[-1:]
          msg = libsafety_py.make_CANPacket(0x283, 0, bytes(dat))
          self.assertEqual(not bad and not stock_longitudinal, self._tx(msg))

  # Only allow LTA msgs with no actuation
  def test_lta_steer_cmd(self):
    for engaged, req, req2, torque_wind_down, angle in itertools.product([True, False],
                                                                  [0, 1], [0, 1],
                                                                  [0, 50, 100],
                                                                  np.linspace(-20, 20, 5)):
      self.safety.set_controls_allowed(engaged)

      should_tx = not req and not req2 and angle == 0 and torque_wind_down == 0
      self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)),
                       f"{req=} {req2=} {angle=} {torque_wind_down=}")

  def test_rx_hook(self):
    # checksum checks
    for msg in ["trq", "pcm"]:
      self.safety.set_controls_allowed(1)
      if msg == "trq":
        msg = self._torque_meas_msg(0)
      if msg == "pcm":
        msg = self._pcm_status_msg(True)
      self.assertTrue(self._rx(msg))
      msg[0].data[4] = 0
      msg[0].data[5] = 0
      msg[0].data[6] = 0
      msg[0].data[7] = 0
      self.assertFalse(self._rx(msg))
      self.assertFalse(self.safety.get_controls_allowed())

    # quality flag tests
    msg = self._speed_msg(0)
    self.assertTrue(self._rx(msg))

    msg = self._speed_msg(0, quality_flag=False)
    self.assertFalse(self._rx(msg))

  def test_vehicle_speed_measurements(self):
    # OVERRIDDEN: 72.22_ is the max speed in m/s
    self._common_measurement_test(self._speed_msg, 0, 259 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)


class TestToyotaSafetyTorque(TestToyotaSafetyBase, common.MotorTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  MAX_RATE_UP = 15
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [1500]
  MAX_RT_DELTA = 450
  MAX_TORQUE_ERROR = 350
  TORQUE_MEAS_TOLERANCE = 1  # toyota safety adds one to be conservative for rounding

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 17
  MAX_INVALID_STEERING_FRAMES = 1

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE


class TestToyotaSafetyAngle(TestToyotaSafetyBase, common.AngleSteeringSafetyTest):

  # Angle control limits
  STEER_ANGLE_MAX = 94.9461  # deg
  DEG_TO_CAN = 17.452007  # 1 / 0.0573 deg to can

  ANGLE_RATE_BP = [5., 25., 25.]
  ANGLE_RATE_UP = [0.3, 0.15, 0.15]  # windup limit
  ANGLE_RATE_DOWN = [0.36, 0.26, 0.26]  # unwind limit

  MAX_LTA_ANGLE = 94.9461  # PCS faults if commanding above this, deg
  MAX_MEAS_TORQUE = 1500  # max allowed measured EPS torque before wind down
  MAX_LTA_DRIVER_TORQUE = 150  # max allowed driver torque before wind down

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.LTA

  # Only allow LKA msgs with no actuation
  def test_lka_steer_cmd(self):
    for engaged, steer_req, torque in itertools.product([True, False],
                                                        [0, 1],
                                                        np.linspace(-1500, 1500, 7)):
      self.safety.set_controls_allowed(engaged)
      torque = int(torque)
      self.safety.set_rt_torque_last(torque)
      self.safety.set_torque_meas(torque, torque)
      self.safety.set_desired_torque_last(torque)

      should_tx = not steer_req and torque == 0
      self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(torque, steer_req)))

  def test_lta_steer_cmd(self):
    """
    Tests the LTA steering command message
    controls_allowed:
    * STEER_REQUEST and STEER_REQUEST_2 do not mismatch
    * TORQUE_WIND_DOWN is only set to 0 or 100 when STEER_REQUEST and STEER_REQUEST_2 are both 1
    * Full torque messages are blocked if either EPS torque or driver torque is above the threshold

    not controls_allowed:
    * STEER_REQUEST, STEER_REQUEST_2, and TORQUE_WIND_DOWN are all 0
    """
    for controls_allowed in (True, False):
      for angle in np.arange(-90, 90, 1):
        self.safety.set_controls_allowed(controls_allowed)
        self._reset_angle_measurement(angle)
        self._set_prev_desired_angle(angle)

        self.assertTrue(self._tx(self._lta_msg(0, 0, angle, 0)))
        if controls_allowed:
          # Test the two steer request bits and TORQUE_WIND_DOWN torque wind down signal
          for req, req2, torque_wind_down in itertools.product([0, 1], [0, 1], [0, 50, 100]):
            mismatch = not (req or req2) and torque_wind_down != 0
            should_tx = req == req2 and (torque_wind_down in (0, 100)) and not mismatch
            self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)))

        else:
          # Controls not allowed
          for req, req2, torque_wind_down in itertools.product([0, 1], [0, 1], [0, 50, 100]):
            should_tx = not (req or req2) and torque_wind_down == 0
            self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)))

    # Test max EPS torque and driver override thresholds (independent of angle, test a few representative angles)
    for angle in (-89, 0, 89):
      self.safety.set_controls_allowed(True)
      self._reset_angle_measurement(angle)
      self._set_prev_desired_angle(angle)

      cases = itertools.product(
        (0, self.MAX_MEAS_TORQUE - 1, self.MAX_MEAS_TORQUE, self.MAX_MEAS_TORQUE + 1, self.MAX_MEAS_TORQUE * 2),
        (0, self.MAX_LTA_DRIVER_TORQUE - 1, self.MAX_LTA_DRIVER_TORQUE, self.MAX_LTA_DRIVER_TORQUE + 1, self.MAX_LTA_DRIVER_TORQUE * 2)
      )

      for eps_torque, driver_torque in cases:
        for sign in (-1, 1):
          for _ in range(6):
            self._rx(self._torque_meas_msg(sign * eps_torque, sign * driver_torque))

          # Toyota adds 1 to EPS torque since it is rounded after EPS factor
          should_tx = (eps_torque - 1) <= self.MAX_MEAS_TORQUE and driver_torque <= self.MAX_LTA_DRIVER_TORQUE
          self.assertEqual(should_tx, self._tx(self._lta_msg(1, 1, angle, 100)))
          self.assertTrue(self._tx(self._lta_msg(1, 1, angle, 0)))  # should tx if we wind down torque

  def test_angle_measurements(self):
    """
    * Tests angle meas quality flag dictates whether angle measurement is parsed, and if rx is valid
    * Tests rx hook correctly clips the angle measurement, since it is to be compared to LTA cmd when inactive
    """
    for steer_angle_initializing in (True, False):
      for angle in np.arange(0, self.STEER_ANGLE_MAX * 2, 1):
        # If init flag is set, do not rx or parse any angle measurements
        for a in (angle, -angle, 0, 0, 0, 0):
          self.assertEqual(not steer_angle_initializing,
                           self._rx(self._angle_meas_msg(a, steer_angle_initializing)))

        final_angle = 0 if steer_angle_initializing else round(angle * self.DEG_TO_CAN)
        self.assertEqual(self.safety.get_angle_meas_min(), -final_angle)
        self.assertEqual(self.safety.get_angle_meas_max(), final_angle)

        self._rx(self._angle_meas_msg(0))
        self.assertEqual(self.safety.get_angle_meas_min(), -final_angle)
        self.assertEqual(self.safety.get_angle_meas_max(), 0)

        self._rx(self._angle_meas_msg(0))
        self.assertEqual(self.safety.get_angle_meas_min(), 0)
        self.assertEqual(self.safety.get_angle_meas_max(), 0)


class TestToyotaAltBrakeSafety(TestToyotaSafetyTorque):

  DBC = "toyota_new_mc_pt_generated"

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.ALT_BRAKE

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("BRAKE_MODULE", 0, values)

  # No LTA message in the DBC
  def test_lta_steer_cmd(self):
    pass


class TestToyotaStockLongitudinalBase(TestToyotaSafetyBase):

  TX_MSGS = TOYOTA_COMMON_TX_MSGS
  # Base addresses minus ACC_CONTROL (0x343)
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191]}

  LONGITUDINAL = False

  def test_diagnostics(self, stock_longitudinal: bool = True, ecu_disabled: bool = True):
    super().test_diagnostics(stock_longitudinal=stock_longitudinal, ecu_disabled=ecu_disabled)

  def test_block_aeb(self, stock_longitudinal: bool = True):
    super().test_block_aeb(stock_longitudinal=stock_longitudinal)

  def test_acc_cancel(self):
    """
      Regardless of controls allowed, never allow ACC_CONTROL if cancel bit isn't set
    """
    for controls_allowed in [True, False]:
      self.safety.set_controls_allowed(controls_allowed)
      for accel in np.arange(self.MIN_ACCEL - 1, self.MAX_ACCEL + 1, 0.1):
        self.assertFalse(self._tx(self._accel_msg_343(accel)))
        should_tx = np.isclose(accel, self.INACTIVE_ACCEL, atol=0.0001)
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel, cancel_req=1)))


class TestToyotaStockLongitudinalTorque(TestToyotaStockLongitudinalBase, TestToyotaSafetyTorque):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL


class TestToyotaStockLongitudinalAngle(TestToyotaStockLongitudinalBase, TestToyotaSafetyAngle):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.LTA


class TestToyotaSecOcSafetyBase(TestToyotaSafetyBase):

  DBC = "toyota_secoc_pt_generated"

  TX_MSGS = TOYOTA_SECOC_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x131)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x191, 0x412, 0x131]}

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.SECOC

  def test_diagnostics(self, ecu_disabled: bool = False):
    super().test_diagnostics(ecu_disabled=ecu_disabled)

  # This platform also has alternate brake and PCM messages, but same naming in the DBC, so same packers work

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL_USER": gas}
    return self.packer.make_can_msg_safety("GAS_PEDAL", 0, values)

  # This platform sends both STEERING_LTA (same as other Toyota) and STEERING_LTA_2 (SecOC signed)
  # STEERING_LTA is checked for no-actuation by the base class, STEERING_LTA_2 is checked for no-actuation below

  def _lta_2_msg(self, req, req2, angle_cmd, torque_wind_down=100):
    values = {"STEER_REQUEST": req, "STEER_REQUEST_2": req2, "STEER_ANGLE_CMD": angle_cmd}
    return self.packer.make_can_msg_safety("STEERING_LTA_2", 0, values)

  def test_lta_2_steer_cmd(self):
    for engaged, req, req2, angle in itertools.product([True, False], [0, 1], [0, 1], np.linspace(-20, 20, 5)):
      self.safety.set_controls_allowed(engaged)

      should_tx = not req and not req2 and angle == 0
      self.assertEqual(should_tx, self._tx(self._lta_2_msg(req, req2, angle)), f"{req=} {req2=} {angle=}")

  def _accel_msg_183(self, accel):
    values = {"ACCEL_CMD": accel}
    return self.packer.make_can_msg_safety("ACC_CONTROL_2", 0, values)

  def _accel_msg(self, accel, cancel_req=0):
    return self._accel_msg_183(accel)


class TestToyotaSecOcSafetyStockLongitudinal(TestToyotaSecOcSafetyBase, TestToyotaStockLongitudinalBase):

  @property
  def SAFETY_PARAM(self):
    return self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.SECOC


class TestToyotaSecOcSafety(TestToyotaSecOcSafetyBase):

  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x131, 0x343, 0x183)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x191, 0x412, 0x131, 0x343, 0x183]}

  @unittest.skip("test not applicable for cars without a DSU")
  def test_block_aeb(self, stock_longitudinal: bool = False):
    pass

  def test_343_actuation_blocked(self):
    """
    For SecOC cars, longitudinal acceleration must be sent in ACC_CONTROL_2, but all other ACC
    data remains in ACC_CONTROL. Verify no actuation is sent via ACC_CONTROL.
    """
    for controls_allowed in [True, False]:
      self.safety.set_controls_allowed(controls_allowed)
      for accel in np.arange(self.MIN_ACCEL - 1, self.MAX_ACCEL + 1, 0.1):
        should_tx = np.isclose(accel, self.INACTIVE_ACCEL, atol=0.0001)
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel)))
        self.assertEqual(should_tx, self._tx(self._accel_msg_343(accel, cancel_req=1)))


def build_signer_requests(seq: int, request: bytes):
  return toyotacan.create_tss3_signer_requests(TSS3_PACKER, 0, seq, request)


def build_host_application(lat_active: bool = False, angle_raw: int = 0, accel: float = 0.0, request_sequence: int = 0,
                           stock: bytes | None = None) -> bytes:
  stock_values = None
  if stock is not None:
    parser = CANParser(TSS3_DBC, [("CONTROL_REQUEST", float("nan"))], 2)
    parser.update([(0, [(0x08A, stock[:28] + TSS3_TRAILER, 2)])])
    stock_values = dict(parser.vl["CONTROL_REQUEST"])
  values = toyotacan.create_tss3_control_request_values(stock_values, lat_active, angle_raw, True, accel, 0.0, request_sequence)
  return TSS3_PACKER.make_can_msg("CONTROL_REQUEST", 0, values)[1][:28]


def fix_toyota_checksum(msg):
  address, data, bus = msg
  payload = bytearray(data)
  payload[-1] = (address + (address >> 8) + len(payload) + sum(payload[:-1])) & 0xFF
  return address, bytes(payload), bus


# FRC CONTROL_REQUESTs recorded around a PCS event: stock ACC braking, then both PCS braking request IDs (34, 33)
TSS3_FRC_08A = bytes.fromhex("0000000880002d47f0605ef0607fff007fff000c4000100000000a00fc472d50")
TSS3_FRC_PCS_08A = (bytes.fromhex("0000000cc0002d8bf0605ef0607fff007fff000b4000100000000b003a194b8d"),
                    bytes.fromhex("00000008c0002d87f0605ef0607fff007fff000d400030000000130006f38fbe"))


class Tss3SafetyHelpers:
  signer_seq = 0

  @staticmethod
  def _admin_msg(arm: bool):
    return libsafety_py.make_CANPacket(0x777, 1, bytes((7, 0xC9, 0xA8, int(arm), 0, 0, 0, 0)))

  def _rx_frc_08a(self, request: bytes):
    msg = libsafety_py.make_CANPacket(0x08A, 2, request)
    msg[0].fd = 1
    self.assertTrue(self.safety.safety_rx_hook(msg))

  def test_frc_pcs_request_is_forwarded(self):
    stock_longitudinal = bool(self.SAFETY_PARAM & ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    for pcs in TSS3_FRC_PCS_08A:
      with self.subTest(pcs=pcs.hex()):
        self._rx_frc_08a(TSS3_FRC_08A)
        self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))
        request = build_host_application(stock=TSS3_FRC_08A if stock_longitudinal else None)
        self.assertTrue(self._request(request))
        self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

        # openpilot's CONTROL_REQUEST is released until the FRC stops requesting PCS braking
        self._rx_frc_08a(pcs)
        self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)
        self.assertFalse(self._publish(request))
        self.assertFalse(self.safety.safety_tx_hook(self._admin_msg(True)))
        self._rx_frc_08a(pcs)
        self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), 0)

        self._rx_frc_08a(TSS3_FRC_08A)
        self.assertTrue(self.safety.safety_tx_hook(self._admin_msg(True)))
        self.assertEqual(self.safety.safety_fwd_hook(2, 0x08A), -1)

        # init forgets the PCS request, stock longitudinal also needs a new FRC request to arm
        self._rx_frc_08a(pcs)
        self._reset_safety_hooks()
        self.safety.set_timer(0)
        self.assertEqual(self.safety.safety_tx_hook(self._admin_msg(True)), not stock_longitudinal)

  @staticmethod
  def _control_request_msg(request: bytes, fd: bool = True):
    msg = libsafety_py.make_CANPacket(0x08A, 0, request + TSS3_TRAILER)
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

  DBC = TSS3_DBC
  SAFETY_MODEL = CarParams.SafetyModel.toyota
  SAFETY_PARAM = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.TSS3

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
    super().setUp()
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._admin_msg(True)))
    self.angle_cmd_count = 0

    fingerprint = {bus: {} for bus in range(8)}
    self.CP = CarInterface.get_params(CAR.TOYOTA_CAMRY_TSS3, fingerprint, [], True, False, False)
    self.VM = VehicleModel(get_safety_CP())
    self.params = CarControllerParams(self.CP)
    self.params.STEER_STEP = int(1 / (0.01 * self.LATERAL_FREQUENCY))

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

  def _application(self, *, angle_raw: int = 0, lat_active: bool = False, accel: float = 0.0, request_sequence: int = 0) -> bytes:
    return build_host_application(lat_active=lat_active, angle_raw=angle_raw, accel=accel, request_sequence=request_sequence)

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

  def test_angle_reference_restarts_after_request_gap(self):
    # after 100 ms without requests, the rate limit restarts from the measured angle, clamped to the command range
    self._reset_speed_measurement(5.)
    self.safety.set_controls_allowed(True)
    t = 0
    for measured, gap, restarted in ((30., 100_000, False), (30., 100_001, True), (120., 100_001, True), (-120., 100_001, True)):
      with self.subTest(measured=measured, gap=gap):
        t += 1_000_000
        self.safety.set_timer(t)
        self._reset_angle_measurement(0.)
        self.assertTrue(self._request(self._application(lat_active=True)))

        self._reset_angle_measurement(measured)
        angle_raw = max(min(self.safety.get_angle_meas_max(), 1745), -1745)
        self.safety.set_timer(t + gap)
        self.assertEqual(self._request(self._application(angle_raw=angle_raw, lat_active=True)), restarted)

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 71.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_requests_are_checked_not_publications(self):
    # a lost signer response skips a generation, the published frames are still approved requests
    self._reset_speed_measurement(10.)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    delta = self._max_delta_raw(9.)
    generations = [self._application(angle_raw=i * delta, lat_active=True, request_sequence=i + 1) for i in range(3)]
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
    flips = [(index, 0x01) for index in (0, 1, 2, 3, 4, 5, 6, 7, 13, 14, 15, 16, 17, 20, 22, 23, 25, 27)] + [(21, 0x40), (21, 0x80)]
    for index, bit in flips:
      data = bytearray(request)
      data[index] ^= bit
      self.assertFalse(self._request(bytes(data)), (index, bit))
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
      data = bytearray((7, 0xC9, 0xA8, 1, 0, 0, 0, 0))
      data[index] ^= 1
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x777, 1, bytes(data))), index)
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

    # bytes 1 and 3 carry the car's state, the others must stay clear
    for index in range(1, 7):
      data = bytearray(cancel_data)
      data[index] ^= 1
      _, data, _ = fix_toyota_checksum((0x101, bytes(data), 2))
      self.assertEqual(self._tx(libsafety_py.make_CANPacket(0x101, 2, data)), index in (1, 3), index)

  def test_brake_cancel_from_recorded_state(self):
    # BRAKE_MODULE frames recorded on a Camry, released and pressed
    for frame in ("800000000000008a", "801200000000009c", "800000020000008c", "8800000400000096"):
      with self.subTest(frame=frame):
        parser = CANParser(TSS3_DBC, [("BRAKE_MODULE", float("nan"))], 0)
        parser.update([(0, [(0x101, bytes.fromhex(frame), 0)])])
        address, dat, bus = toyotacan.create_tss3_brake_cancel_command(TSS3_PACKER, dict(parser.vl["BRAKE_MODULE"]), 2)
        self.assertTrue(self._tx(libsafety_py.make_CANPacket(address, bus, dat)))

  def test_lkas_hud_classic_can_only(self):
    for fd in (False, True):
      msg = libsafety_py.make_CANPacket(0x412, 0, bytes(8))
      msg[0].fd = fd
      self.assertEqual(self._tx(msg), not fd)

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


class TestToyotaTss3CamryStockLongitudinalSafety(Tss3SafetyHelpers, common.SafetyTestBase):

  SAFETY_MODEL = CarParams.SafetyModel.toyota
  SAFETY_PARAM = EPS_SCALE[CAR.TOYOTA_CAMRY_TSS3] | ToyotaSafetyFlags.TSS3 | ToyotaSafetyFlags.STOCK_LONGITUDINAL

  STOCK_08A = bytes.fromhex("0000000880002d47fe462afe467fff007fffff35c000100064003c005db7797f")

  def setUp(self):
    super().setUp()
    self.safety.set_timer(0)

  def _rx_stock(self, stock: bytes):
    self._rx_frc_08a(stock)

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

    flips = [(index, 0x01) for index in (3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 17, 20, 22, 23, 27)] + [(21, 0x80), (26, 0x80)]
    for index, bit in flips:
      with self.subTest(index=index, bit=bit):
        request = bytearray(self._host_request())
        request[index] ^= bit
        self.assertFalse(self._request(bytes(request)))

    request = self._host_request()
    self.assertTrue(self._request(request))
    self.assertTrue(self._publish(request))

  def test_only_latest_frc_requests_match(self):
    frc = []
    for i in range(3):
      stock = bytearray(self.STOCK_08A)
      stock[8] ^= i
      frc.append(bytes(stock))

    self._rx_stock(frc[0])
    self._rx_stock(frc[1])
    self.assertTrue(self._request(self._host_request(frc[0])))
    self.assertTrue(self._request(self._host_request(frc[1])))

    self._rx_stock(frc[2])
    self.assertFalse(self._request(self._host_request(frc[0])))
    self.assertTrue(self._request(self._host_request(frc[1])))
    self.assertTrue(self._request(self._host_request(frc[2])))

  def test_init_forgets_frc_requests(self):
    stale = bytearray(self.STOCK_08A)
    stale[8] ^= 1
    self._rx_stock(self.STOCK_08A)
    self._rx_stock(bytes(stale))

    self._reset_safety_hooks()
    self._rx_stock(self.STOCK_08A)
    self.assertFalse(self._request(self._host_request(bytes(stale))))
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
