#!/usr/bin/env python3
import unittest
import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerPanda


class TestBydSafety(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest):

  TX_MSGS = [[0x1E2, 0], [0x316, 0], [0x32E, 0]]
  STANDSTILL_THRESHOLD = 0
  GAS_PRESSED_THRESHOLD = 1
  RELAY_MALFUNCTION_ADDRS = {0: (0x1E2, 0x316, 0x32E)}
  FWD_BLACKLISTED_ADDRS = {0: [], 2: [0x1E2, 0x316, 0x32E]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAIN_BUS = 0
  CAM_BUS = 2

  # Angle control limits
  STEER_ANGLE_MAX = 300
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [6., 4., 3.]  # windup limit
  ANGLE_RATE_DOWN = [8., 6., 4.]  # unwind limit

  def setUp(self):
    self.packer = CANPackerPanda("byd_general_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"STEER_ANGLE": angle, "STEER_REQ": 1 if enabled else 0}
    return self.packer.make_can_msg_panda("STEERING_MODULE_ADAS", 0, values)

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE_2": angle}
    return self.packer.make_can_msg_panda("STEER_MODULE_2", self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_CONTROLLABLE_AND_ON": enable}
    return self.packer.make_can_msg_panda("ACC_CMD", self.CAM_BUS, values)

  def _speed_msg(self, speed):
    values = {"WHEELSPEED_%s" % s: speed * 3.6 for s in ["BL", "FL"]}
    return self.packer.make_can_msg_panda("WHEEL_SPEED", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL": brake}
    return self.packer.make_can_msg_panda("PEDAL", self.MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_panda("PEDAL", self.MAIN_BUS, values)

  def _acc_button_cmd(self, cancel=0, _set=0, res=0):
    values = {"ACC_ON_BTN": cancel, "SET_BTN": _set, "RES_BTN": res}
    return self.packer.make_can_msg_panda("PCM_BUTTONS", self.MAIN_BUS, values)

  def test_acc_buttons(self):
    btns = [
      ("cancel", False),
      ("_set", True),
      ("res", True),
    ]
    for btn, should_tx in btns:
      args = {} if btn is None else {btn: 1}
      self._rx(self._acc_button_cmd(**args))
      self.assertEqual(should_tx, self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
