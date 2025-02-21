#!/usr/bin/env python3
from parameterized import parameterized_class
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.safety import Safety
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase


class TestHyundaiCanfdBase(HyundaiButtonBase, common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  SAFETY_PARAM = 0
  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

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

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdBase":
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerPanda("hyundai_canfd")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(Safety.SAFETY_HYUNDAI_CANFD, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _torque_driver_msg(self, torque):
    values = {"STEERING_COL_TORQUE": torque}
    return self.packer.make_can_msg_panda("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"TORQUE_REQUEST": torque, "STEER_REQ": steer_req}
    return self.packer.make_can_msg_panda(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHEEL_SPEED_{i}": speed * 0.03125 for i in range(1, 5)}
    return self.packer.make_can_msg_panda("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_panda("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_panda(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_panda("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_panda("CRUISE_BUTTONS", bus, values)


class HyundaiCanfdAltButtonsMixin:
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    cls.SAFETY_PARAM |= HyundaiSafetyFlags.FLAG_HYUNDAI_CANFD_ALT_BUTTONS
    cls.TX_MSGS.append([0x1AA, cls.BUTTONS_TX_BUS])

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    msg = self.packer.make_can_msg_panda("CRUISE_BUTTONS_ALT", bus, values)
    return msg
  def _lkas_button_msg(self, enabled):
    values = {"LFA_BTN": enabled}
    return self.packer.make_can_msg_panda("CRUISE_BUTTONS_ALT", self.PT_BUS, values)


class TestHyundaiCanfdHDA1Base(TestHyundaiCanfdBase):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 2], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A,)}  # LFA
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0, 0x161, 0x162]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdHDA1", "TestHyundaiCanfdHDA1AltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest


@parameterized_class([
  # Radar SCC, test with long flag to ensure flag is not respected until it is supported
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_LONG},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS |
                                                                                  HyundaiSafetyFlags.FLAG_HYUNDAI_LONG},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_HYBRID_GAS |
                                                                                      HyundaiSafetyFlags.FLAG_HYUNDAI_LONG},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS |
                                                                                  HyundaiSafetyFlags.FLAG_HYUNDAI_CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_HYBRID_GAS |
                                                                                      HyundaiSafetyFlags.FLAG_HYUNDAI_CAMERA_SCC},
])
class TestHyundaiCanfdHDA1(TestHyundaiCanfdHDA1Base):
  pass


@parameterized_class([
  # Radar SCC, test with long flag to ensure flag is not respected until it is supported
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_LONG},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS |
                                                                                  HyundaiSafetyFlags.FLAG_HYUNDAI_LONG},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_HYBRID_GAS |
                                                                                      HyundaiSafetyFlags.FLAG_HYUNDAI_LONG},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS |
                                                                                  HyundaiSafetyFlags.FLAG_HYUNDAI_CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_HYBRID_GAS |
                                                                                      HyundaiSafetyFlags.FLAG_HYUNDAI_CAMERA_SCC},
])
class TestHyundaiCanfdHDA1AltButtons(HyundaiCanfdAltButtonsMixin, TestHyundaiCanfdHDA1Base):
  def _lkas_button_msg(self, enabled):
    values = {"LFA_BTN": enabled}
    return self.packer.make_can_msg_panda("CRUISE_BUTTONS_ALT", self.PT_BUS, values)


# ********************* HDA2 **********************


# TODO: Handle ICE and HEV configurations.
class TestHyundaiCanfdHDA2EVBase(TestHyundaiCanfdBase):
  PT_BUS = 1
  SCC_BUS = 1
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  FWD_BUS_LOOKUP = {0: 2, 2: 0}


class TestHyundaiCanfdHDA2EV(TestHyundaiCanfdHDA2EVBase):
  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}
  STEER_MSG = "LKAS"
  SAFETY_PARAM = HyundaiSafetyFlags.FLAG_HYUNDAI_CANFD_HDA2 | HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS
  BUTTONS_TX_BUS = 1


class TestHyundaiCanfdHDA2EVAltSteering(TestHyundaiCanfdHDA2EVBase):
  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}
  STEER_MSG = "LKAS_ALT"
  SAFETY_PARAM = (HyundaiSafetyFlags.FLAG_HYUNDAI_CANFD_HDA2 |
                  HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS |
                  HyundaiSafetyFlags.FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING)


class TestHyundaiCanfdHDA2EVAltButtons(HyundaiCanfdAltButtonsMixin, TestHyundaiCanfdHDA2EV):
  pass

class TestHyundaiCanfdHDA2EVAltSteeringAltButtons(HyundaiCanfdAltButtonsMixin, TestHyundaiCanfdHDA2EVAltSteering):
  pass


# ********************* LONG **********************


class TestHyundaiCanfdHDA2LongEV(HyundaiLongitudinalBase, TestHyundaiCanfdHDA2EV):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50,), 1: (0x1a0,)}  # LKAS, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1
  SAFETY_PARAM = (HyundaiSafetyFlags.FLAG_HYUNDAI_CANFD_HDA2 |
                  HyundaiSafetyFlags.FLAG_HYUNDAI_LONG |
                  HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS)

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_panda("SCC_CONTROL", 1, values)


class TestHyundaiCanfdHDA2LongEVAltSteering(HyundaiLongitudinalBase, TestHyundaiCanfdHDA2EVAltSteering):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x110,), 1: (0x1a0,)}  # LKAS, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1
  SAFETY_PARAM = (HyundaiSafetyFlags.FLAG_HYUNDAI_CANFD_HDA2 |
                  HyundaiSafetyFlags.FLAG_HYUNDAI_LONG |
                  HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS |
                  HyundaiSafetyFlags.FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING)

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_panda("SCC_CONTROL", 1, values)


# Tests HDA1 longitudinal for ICE, hybrid, EV
@parameterized_class([
  # Camera SCC is the only supported configuration for HDA1 longitudinal, TODO: allow radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_LONG},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_LONG | HyundaiSafetyFlags.FLAG_HYUNDAI_EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SAFETY_PARAM": HyundaiSafetyFlags.FLAG_HYUNDAI_LONG | HyundaiSafetyFlags.FLAG_HYUNDAI_HYBRID_GAS},
])
class TestHyundaiCanfdHDA1Long(HyundaiLongitudinalBase, TestHyundaiCanfdHDA1Base):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x161, 0x162]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1a0)}  # LFA, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  STEER_MSG = "LFA"
  STEER_BUS = 0
  SCC_BUS = 2

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    cls.SAFETY_PARAM |= HyundaiSafetyFlags.FLAG_HYUNDAI_CAMERA_SCC
    if cls.__name__ == "TestHyundaiCanfdHDA1Long":
      cls.safety = None
      raise unittest.SkipTest

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_panda("SCC_CONTROL", 0, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_panda("SCC_CONTROL", 0, values)

  # no knockout
  def test_tester_present_allowed(self):
    pass


if __name__ == "__main__":
  unittest.main()
