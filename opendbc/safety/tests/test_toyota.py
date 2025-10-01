#!/usr/bin/env python3
from parameterized import parameterized_class
import numpy as np
import random
import unittest
import itertools

from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.sunnypilot.car.toyota.values import ToyotaSafetyFlagsSP
from opendbc.car.structs import CarParams
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda
from opendbc.safety.tests.gas_interceptor_common import GasInterceptorSafetyTest

TOYOTA_COMMON_TX_MSGS = [[0x2E4, 0], [0x191, 0], [0x412, 0], [0x343, 0], [0x1D2, 0]]  # LKAS + LTA + ACC & PCM cancel cmds
TOYOTA_SECOC_TX_MSGS = [[0x131, 0], [0x183, 0]] + TOYOTA_COMMON_TX_MSGS
TOYOTA_COMMON_LONG_TX_MSGS = [[0x283, 0], [0x2E6, 0], [0x2E7, 0], [0x33E, 0], [0x344, 0], [0x365, 0], [0x366, 0], [0x4CB, 0],  # DSU bus 0
                              [0x128, 1], [0x141, 1], [0x160, 1], [0x161, 1], [0x470, 1],  # DSU bus 1
                              [0x411, 0],  # PCS_HUD
                              [0x750, 0]]  # radar diagnostic address
GAS_INTERCEPTOR_TX_MSGS = [[0x200, 0]]

UNSUPPORTED_DSU = [
  {"SAFETY_PARAM_SP": ToyotaSafetyFlagsSP.DEFAULT},
  {"SAFETY_PARAM_SP": ToyotaSafetyFlagsSP.UNSUPPORTED_DSU},
]


class TestToyotaSafetyBase(common.PandaCarSafetyTest, common.LongitudinalAccelSafetyTest):

  TX_MSGS = TOYOTA_COMMON_TX_MSGS + TOYOTA_COMMON_LONG_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x343)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191, 0x343]}
  EPS_SCALE = 73

  SAFETY_PARAM_SP: int = 0

  packer: CANPackerPanda
  safety: libsafety_py.Panda

  def _torque_meas_msg(self, torque: int, driver_torque: int | None = None):
    values = {"STEER_TORQUE_EPS": (torque / self.EPS_SCALE) * 100.}
    if driver_torque is not None:
      values["STEER_TORQUE_DRIVER"] = driver_torque
    return self.packer.make_can_msg_panda("STEER_TORQUE_SENSOR", 0, values)

  # Both torque and angle safety modes test with each other's steering commands
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEER_TORQUE_CMD": torque, "STEER_REQUEST": steer_req}
    return self.packer.make_can_msg_panda("STEERING_LKA", 0, values)

  def _angle_meas_msg(self, angle: float, steer_angle_initializing: bool = False):
    # This creates a steering torque angle message. Not set on all platforms,
    # relative to init angle on some older TSS2 platforms. Only to be used with LTA
    values = {"STEER_ANGLE": angle, "STEER_ANGLE_INITIALIZING": int(steer_angle_initializing)}
    return self.packer.make_can_msg_panda("STEER_TORQUE_SENSOR", 0, values)

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    return self._lta_msg(int(enabled), int(enabled), angle, torque_wind_down=100 if enabled else 0)

  def _lta_msg(self, req, req2, angle_cmd, torque_wind_down=100):
    values = {"STEER_REQUEST": req, "STEER_REQUEST_2": req2, "STEER_ANGLE_CMD": angle_cmd, "TORQUE_WIND_DOWN": torque_wind_down}
    return self.packer.make_can_msg_panda("STEERING_LTA", 0, values)

  def _accel_msg(self, accel, cancel_req=0):
    values = {"ACCEL_CMD": accel, "CANCEL_REQ": cancel_req}
    return self.packer.make_can_msg_panda("ACC_CONTROL", 0, values)

  def _speed_msg(self, speed):
    values = {("WHEEL_SPEED_%s" % n): speed * 3.6 for n in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_panda("WHEEL_SPEEDS", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_panda("BRAKE_MODULE", 0, values)

  def _user_gas_msg(self, gas):
    cruise_active = self.safety.get_controls_allowed()
    values = {"GAS_RELEASED": not gas, "CRUISE_ACTIVE": cruise_active}
    return self.packer.make_can_msg_panda("PCM_CRUISE", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_ACTIVE": enable}
    return self.packer.make_can_msg_panda("PCM_CRUISE", 0, values)

  def _acc_state_msg(self, enabled):
    msg = "DSU_CRUISE" if self.SAFETY_PARAM_SP & ToyotaSafetyFlagsSP.UNSUPPORTED_DSU else "PCM_CRUISE_2"
    values = {"MAIN_ON": enabled}
    return self.packer.make_can_msg_panda(msg, 0, values)

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


class TestToyotaSafetyGasInterceptorBase(GasInterceptorSafetyTest, TestToyotaSafetyBase):

  TX_MSGS = TOYOTA_COMMON_TX_MSGS + TOYOTA_COMMON_LONG_TX_MSGS + GAS_INTERCEPTOR_TX_MSGS
  INTERCEPTOR_THRESHOLD = 805

  def setUp(self):
    super().setUp()
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | ToyotaSafetyFlagsSP.GAS_INTERCEPTOR)
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.safety.get_current_safety_param())
    self.safety.init_tests()

  def test_stock_longitudinal(self):
    # If stock longitudinal is set, the gas interceptor safety param should not be respected
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | ToyotaSafetyFlagsSP.GAS_INTERCEPTOR)
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.safety.get_current_safety_param() | ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    self.safety.init_tests()

    # Spot check a few gas interceptor tests: (1) reading interceptor,
    # (2) behavior around interceptor, and (3) txing interceptor msgs
    for test in (self.test_prev_gas_interceptor, self.test_no_disengage_on_gas_interceptor,
                 self.test_gas_interceptor_safety_check):
      with self.subTest(test=test.__name__):
        with self.assertRaises(AssertionError):
          test()


@parameterized_class(UNSUPPORTED_DSU)
class TestToyotaSafetyTorque(TestToyotaSafetyBase, common.MotorTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  MAX_RATE_UP = 15
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [1500]
  MAX_RT_DELTA = 450
  MAX_TORQUE_ERROR = 350
  TORQUE_MEAS_TOLERANCE = 1  # toyota safety adds one to be conservative for rounding

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 18
  MAX_INVALID_STEERING_FRAMES = 1

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestToyotaSafetyTorque":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerPanda("toyota_nodsu_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.EPS_SCALE)
    self.safety.init_tests()


@parameterized_class(UNSUPPORTED_DSU)
class TestToyotaSafetyTorqueGasInterceptor(TestToyotaSafetyGasInterceptorBase, TestToyotaSafetyTorque):

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestToyotaSafetyTorqueGasInterceptor":
      cls.safety = None
      raise unittest.SkipTest


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

  def setUp(self):
    self.packer = CANPackerPanda("toyota_nodsu_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.EPS_SCALE | ToyotaSafetyFlags.LTA)
    self.safety.init_tests()

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

          # Test max EPS torque and driver override thresholds
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

        else:
          # Controls not allowed
          for req, req2, torque_wind_down in itertools.product([0, 1], [0, 1], [0, 50, 100]):
            should_tx = not (req or req2) and torque_wind_down == 0
            self.assertEqual(should_tx, self._tx(self._lta_msg(req, req2, angle, torque_wind_down)))

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


class TestToyotaSafetyAngleGasInterceptor(TestToyotaSafetyGasInterceptorBase, TestToyotaSafetyAngle):
  pass


@parameterized_class(UNSUPPORTED_DSU)
class TestToyotaAltBrakeSafety(TestToyotaSafetyTorque):

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestToyotaAltBrakeSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerPanda("toyota_new_mc_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.EPS_SCALE | ToyotaSafetyFlags.ALT_BRAKE)
    self.safety.init_tests()

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_panda("BRAKE_MODULE", 0, values)

  # No LTA message in the DBC
  def test_lta_steer_cmd(self):
    pass


@parameterized_class(UNSUPPORTED_DSU)
class TestToyotaAltBrakeSafetyGasInterceptor(TestToyotaSafetyGasInterceptorBase, TestToyotaAltBrakeSafety):

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestToyotaAltBrakeSafetyGasInterceptor":
      cls.safety = None
      raise unittest.SkipTest

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
        self.assertFalse(self._tx(self._accel_msg(accel)))
        should_tx = np.isclose(accel, 0, atol=0.0001)
        self.assertEqual(should_tx, self._tx(self._accel_msg(accel, cancel_req=1)))


@parameterized_class(UNSUPPORTED_DSU)
class TestToyotaStockLongitudinalTorque(TestToyotaStockLongitudinalBase, TestToyotaSafetyTorque):

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestToyotaStockLongitudinalTorque":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerPanda("toyota_nodsu_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL)
    self.safety.init_tests()


class TestToyotaStockLongitudinalAngle(TestToyotaStockLongitudinalBase, TestToyotaSafetyAngle):

  def setUp(self):
    self.packer = CANPackerPanda("toyota_nodsu_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota,
                                 self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.LTA)
    self.safety.init_tests()


class TestToyotaSecOcSafetyBase(TestToyotaSafetyBase):

  TX_MSGS = TOYOTA_SECOC_TX_MSGS
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x191, 0x412, 0x131)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x191, 0x412, 0x131]}

  def setUp(self):
    self.packer = CANPackerPanda("toyota_secoc_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota,
                                 self.EPS_SCALE | ToyotaSafetyFlags.SECOC)
    self.safety.init_tests()

  # 0x283|PRE_COLLISION is not allowed on SecOC platforms
  def test_block_aeb(self, stock_longitudinal: bool = False):
    pass

  def test_diagnostics(self, stock_longitudinal: bool = False, ecu_disabled: bool = False):
    super().test_diagnostics(stock_longitudinal=stock_longitudinal, ecu_disabled=ecu_disabled)

  # This platform also has alternate brake and PCM messages, but same naming in the DBC, so same packers work

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL_USER": gas}
    return self.packer.make_can_msg_panda("GAS_PEDAL", 0, values)

  # This platform sends both STEERING_LTA (same as other Toyota) and STEERING_LTA_2 (SecOC signed)
  # STEERING_LTA is checked for no-actuation by the base class, STEERING_LTA_2 is checked for no-actuation below

  def _lta_2_msg(self, req, req2, angle_cmd, torque_wind_down=100):
    values = {"STEER_REQUEST": req, "STEER_REQUEST_2": req2, "STEER_ANGLE_CMD": angle_cmd}
    return self.packer.make_can_msg_panda("STEERING_LTA_2", 0, values)

  def test_lta_2_steer_cmd(self):
    for engaged, req, req2, angle in itertools.product([True, False], [0, 1], [0, 1], np.linspace(-20, 20, 5)):
      self.safety.set_controls_allowed(engaged)

      should_tx = not req and not req2 and angle == 0
      self.assertEqual(should_tx, self._tx(self._lta_2_msg(req, req2, angle)), f"{req=} {req2=} {angle=}")


class TestToyotaSecOcSafetyStockLongitudinal(TestToyotaSecOcSafetyBase, TestToyotaStockLongitudinalBase):

  def setUp(self):
    self.packer = CANPackerPanda("toyota_secoc_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota,
                                 self.EPS_SCALE | ToyotaSafetyFlags.STOCK_LONGITUDINAL | ToyotaSafetyFlags.SECOC)
    self.safety.init_tests()

  def test_diagnostics(self, stock_longitudinal: bool = True, ecu_disabled: bool = False):
    super().test_diagnostics(stock_longitudinal=stock_longitudinal, ecu_disabled=ecu_disabled)


class TestToyotaSecOcSafety(TestToyotaSecOcSafetyBase):

  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x343, 0x183, 0x191, 0x412, 0x131)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x191, 0x412, 0x131, 0x343, 0x183]}

  def setUp(self):
    self.packer = CANPackerPanda("toyota_secoc_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.EPS_SCALE | ToyotaSafetyFlags.SECOC)
    self.safety.init_tests()

  def _accel_msg_2(self, accel):
    values = {"ACCEL_CMD": accel}
    return self.packer.make_can_msg_panda("ACC_CONTROL_2", 0, values)

  def test_accel_actuation_limits(self, stock_longitudinal=False):
    limits = ((self.MIN_ACCEL, self.MAX_ACCEL, ALTERNATIVE_EXPERIENCE.DEFAULT),
              (self.MIN_ACCEL, self.MAX_ACCEL, ALTERNATIVE_EXPERIENCE.RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX))

    for min_accel, max_accel, alternative_experience in limits:
      # enforce we don't skip over 0 or inactive accel
      for accel in np.concatenate((np.arange(min_accel - 1, max_accel + 1, 0.05), [0, self.INACTIVE_ACCEL])):
        accel = round(accel, 2)  # floats might not hit exact boundary conditions without rounding
        for controls_allowed in [True, False]:
          self.safety.set_controls_allowed(controls_allowed)
          self.safety.set_alternative_experience(alternative_experience)
          # On a SecOC vehicle, we still transmit ACC_CONTROL but the accel value moves to ACC_CONTROL_2
          # Verify that all non-idle accel values in ACC_CONTROL are rejected, verify ACC_CONTROL_2 accel normally
          should_tx_1 = accel == self.INACTIVE_ACCEL
          should_tx_2 = (controls_allowed and min_accel <= accel <= max_accel) or accel == self.INACTIVE_ACCEL
          self.assertEqual(should_tx_1, self._tx(self._accel_msg(accel)))
          self.assertEqual(should_tx_2, self._tx(self._accel_msg_2(accel)))


if __name__ == "__main__":
  unittest.main()
