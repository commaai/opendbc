#!/usr/bin/env python3
import unittest

from opendbc.car.chrysler.values import ChryslerSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


class TestChryslerSafety(common.PandaCarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0], [0x2D9, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x292, 0x2A6, 0x2D9)}
  FWD_BLACKLISTED_ADDRS = {2: [0x292, 0x2A6, 0x2D9]}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [261]
  MAX_RT_DELTA = 112
  MAX_TORQUE_ERROR = 80

  LKAS_ACTIVE_VALUE = 1

  DAS_BUS = 0

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, 0)
    self.safety.init_tests()

  def _button_msg(self, cancel=False, resume=False, accel=False, decel=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume, "ACC_Accel": accel, "ACC_Decel": decel}
    return self.packer.make_can_msg_panda("CRUISE_BUTTONS", self.DAS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable}
    return self.packer.make_can_msg_panda("DAS_3", self.DAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"SPEED_LEFT": speed, "SPEED_RIGHT": speed}
    return self.packer.make_can_msg_panda("SPEED_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_panda("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_panda("ESP_1", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"EPS_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_panda("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": self.LKAS_ACTIVE_VALUE if steer_req else 0}
    return self.packer.make_can_msg_panda("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume/accel/decel only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(accel=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(decel=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

      # invalid: more than one button pressed
      combos = [
        # 2 buttons
        {"cancel": True, "resume": True},
        {"cancel": True, "accel": True},
        {"cancel": True, "decel": True},
        {"resume": True, "accel": True},
        {"resume": True, "decel": True},
        {"accel": True, "decel": True},

        # 3 buttons
        {"cancel": True, "resume": True, "accel": True},
        {"cancel": True, "resume": True, "decel": True},
        {"cancel": True, "accel": True, "decel": True},
        {"resume": True, "accel": True, "decel": True},

        # all 4 buttons
        {"cancel": True, "resume": True, "accel": True, "decel": True},
      ]

      for combo in combos:
        with self.subTest(combo=combo):
          self.assertFalse(self._tx(self._button_msg(**combo)))

  def _lkas_button_msg(self, enabled):
    values = {"TOGGLE_LKAS": enabled}
    return self.packer.make_can_msg_panda("TRACTION_BUTTON", 0, values)


class TestChryslerRamDTSafety(TestChryslerSafety):
  TX_MSGS = [[0xB1, 2], [0xA6, 0], [0xFA, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xA6, 0xFA)}
  FWD_BLACKLISTED_ADDRS = {2: [0xA6, 0xFA]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [350]

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_ram_dt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, ChryslerSafetyFlags.RAM_DT)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_panda("ESP_8", 0, values)

  def _lkas_button_msg(self, enabled):
    values = {"LKAS_Button": enabled}
    return self.packer.make_can_msg_panda("Center_Stack_2", 0, values)


class TestChryslerRamHDSafety(TestChryslerSafety):
  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23A, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x276, 0x275)}
  FWD_BLACKLISTED_ADDRS = {2: [0x275, 0x276]}

  MAX_TORQUE_LOOKUP = [0], [361]
  MAX_RATE_UP = 14
  MAX_RATE_DOWN = 14
  MAX_RT_DELTA = 182

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def setUp(self):
    self.packer = CANPackerPanda("chrysler_ram_hd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, ChryslerSafetyFlags.RAM_HD)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_panda("ESP_8", 0, values)

  def _lkas_button_msg(self, enabled):
    values = {"LKAS_Button": enabled}
    return self.packer.make_can_msg_panda("Center_Stack_2", 0, values)


if __name__ == "__main__":
  unittest.main()
