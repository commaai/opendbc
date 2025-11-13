#!/usr/bin/env python3
from parameterized import parameterized_class
import unittest
import numpy as np

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.hyundai.values import HyundaiSafetyFlags, CAR, HyundaiFlags, CarControllerParams, AVERAGE_ROAD_ROLL
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm, ISO_LATERAL_ACCEL
from parameterized import parameterized
from opendbc.car.hyundai.interface import CarInterface

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


def round_angle(angle_deg: float, can_offset=0):
  scaled = int(angle_deg / 0.1)
  scaled += can_offset
  return scaled * 0.1


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"STEERING_COL_TORQUE": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"TORQUE_REQUEST": torque, "STEER_REQ": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)


class TestHyundaiCanfdTorqueSteering(TestHyundaiCanfdBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE = 270

  MAX_RT_DELTA = 112
  RT_INTERVAL = 250000

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2
  MIN_VALID_STEERING_RT_INTERVAL = 810000  # a ~10% buffer, can send steer up to 110Hz

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdTorqueSteering":
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, 0)
    self.safety.init_tests()


class TestHyundaiCanfdAngleSteering(TestHyundaiCanfdBase, common.AngleSteeringSafetyTest):
  PLATFORMS = {str(platform): platform for platform in CAR if
               platform.config.flags & HyundaiFlags.CANFD_ANGLE_STEERING and not CarInterface.get_non_essential_params(str(platform)).dashcamOnly}

  # Angle control limits
  STEER_ANGLE_MAX = 180  # deg
  DEG_TO_CAN = 10
  MAX_LATERAL_ACCEL=(ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL))
  MAX_LATERAL_JERK=(3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL))

  # Hyundai uses get_max_angle_delta and get_max_angle for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 100  # Hz

  cnt_angle_cmd = 0

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    values = {"ADAS_StrAnglReqVal": angle, "LKAS_ANGLE_ACTIVE": 2 if enabled else 1}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"STEERING_ANGLE": angle}
    return self.packer.make_can_msg_safety("STEERING_SENSORS", self.PT_BUS, values)

  def _get_steer_cmd_angle_max(self, speed):
    raise NotImplementedError("This method can't be used on HKG because we test multiple platforms")

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdAngleSteering":
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def get_vm(self, car_name):
    return VehicleModel(CarInterface.get_non_essential_params(car_name))

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ANGLE_STEERING)
    self.safety.init_tests()

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  @parameterized.expand([(car,) for car in sorted(PLATFORMS)])
  def test_lateral_accel_limit(self, car_name):
    CP = CarInterface.get_non_essential_params(car_name)
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match DI_vehicleSpeed rounding on CAN
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        # at limit (safety tolerance adds 1)
        max_angle = round_angle(get_max_angle_vm(speed, self.get_vm(car_name), CarControllerParams(CP)), 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # 1 unit above limit
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.get_vm(car_name), CarControllerParams(CP)), 3) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)))

  @parameterized.expand([(car,) for car in sorted(PLATFORMS)])
  def test_lateral_jerk_limit(self, car_name):
    self.car_name = car_name
    CP = CarInterface.get_non_essential_params(car_name)
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match DI_vehicleSpeed rounding on CAN
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):  # (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # Stay within limits
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.get_vm(car_name), CarControllerParams(CP))) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.get_vm(car_name), CarControllerParams(CP)), 2) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  @parameterized.expand([(car,) for car in sorted(PLATFORMS)])
  def test_rt_limits(self, car_name):
    self.car_name = car_name
    # TODO: remove and check all safety modes
    if self.LATERAL_FREQUENCY == -1:
      raise unittest.SkipTest("No real time limits")

    # Angle safety enforces real time limits by checking the message send frequency in a 250ms time window
    self.safety.set_timer(0)
    self.safety.set_controls_allowed(True)
    max_rt_msgs = int(self.LATERAL_FREQUENCY * common.RT_INTERVAL / 1e6 * 1.2 + 1)  # 1.2x buffer

    for i in range(max_rt_msgs * 2):
      should_tx = i <= max_rt_msgs
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

    # One under RT interval should do nothing
    self.safety.set_timer(common.RT_INTERVAL - 1)
    for _ in range(5):
      self.assertFalse(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

    # Increment timer and send 1 message to reset RT window
    self.safety.set_timer(common.RT_INTERVAL)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))
    for _ in range(5):
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

  @parameterized.expand([(car,) for car in sorted(PLATFORMS)])
  def test_angle_violation(self, car_name):
    CP = CarInterface.get_non_essential_params(car_name)
    # If violation occurs, angle cmd is blocked until reset to 0. Matches behavior of torque safety modes
    self.safety.set_controls_allowed(True)

    for speed in (0., 1., 5., 10., 15., 50.):
      self._tx(self._angle_cmd_msg(0, True))
      self._reset_speed_measurement(speed)

      for _ in range(20):
        self.assertFalse(self._tx(self._angle_cmd_msg(get_max_angle_vm(max(speed, 1), self.get_vm(car_name), CarControllerParams(CP)), True)))
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdTorqueSteering):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  SAFETY_PARAM: int

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdTorqueSteering):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEVBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringAltEVTorque(TestHyundaiCanfdLKASteeringAltEVBase, TestHyundaiCanfdTorqueSteering):

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringAltEVAngle(TestHyundaiCanfdLKASteeringAltEVBase, TestHyundaiCanfdAngleSteering):

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT | HyundaiSafetyFlags.CANFD_ANGLE_STEERING)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)


# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongBase":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_tester_present_allowed(self, ecu_disable: bool = True):
    super().test_tester_present_allowed(ecu_disable=not self.SAFETY_PARAM & HyundaiSafetyFlags.CAMERA_SCC)


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      cls.safety = None
      raise unittest.SkipTest


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


if __name__ == "__main__":
  unittest.main()
