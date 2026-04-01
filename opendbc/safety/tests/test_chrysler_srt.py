#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestChryslerSrtSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x292, 0x2A6)}
  FWD_BLACKLISTED_ADDRS = {2: [0x292, 0x2A6]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [261]
  MAX_RT_DELTA = 112
  MAX_TORQUE_ERROR = 80

  LKAS_ACTIVE_VALUE = 1
  DAS_BUS = 0

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chryslerSrt, 0)
    self.safety.init_tests()

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.DAS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable}
    return self.packer.make_can_msg_safety("DAS_3", self.DAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"SPEED_LEFT": speed, "SPEED_RIGHT": speed}
    return self.packer.make_can_msg_safety("SPEED_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_safety("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ESP_1", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"EPS_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": self.LKAS_ACTIVE_VALUE if steer_req else 0}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))
      self.assertTrue(self._tx(self._button_msg(cancel=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=False, resume=False)))

  def test_rate_up(self):
    """Torque can increase at MAX_RATE_UP per frame"""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP + 1)))

  def test_rate_down(self):
    """Torque can decrease at MAX_RATE_DOWN per frame"""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(100)
    self.assertTrue(self._tx(self._torque_cmd_msg(100 - self.MAX_RATE_DOWN)))
    self._set_prev_torque(100)
    self.assertFalse(self._tx(self._torque_cmd_msg(100 - self.MAX_RATE_DOWN - 1)))

  def test_max_torque(self):
    """Torque cannot exceed MAX_TORQUE"""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(261)
    self.assertTrue(self._tx(self._torque_cmd_msg(261)))
    self.assertFalse(self._tx(self._torque_cmd_msg(262)))


if __name__ == "__main__":
  unittest.main()