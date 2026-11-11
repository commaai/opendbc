#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common

class TestFcaGiorgio_Safety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  DBC = "fca_giorgio"
  SAFETY_MODEL = CarParams.SafetyModel.fcaGiorgio

  RELAY_MALFUNCTION_ADDRS = {0: (0x117, 0x1F6, 0x547, 0x5A2)}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 2

  TX_MSGS = [[0x117, 0], [0x1F6, 0], [0x547, 0], [0x5A2, 0], [0x2FA, 2]]
  STANDSTILL_THRESHOLD = 0
  FWD_BLACKLISTED_ADDRS = {2: [0x117, 0x1F6, 0x547, 0x5A2]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def _button_msg(self, cancel=False, resume=False):
    values = {"CANCEL": cancel, "RESUME": resume, "UNKNOWN_1": 3, "UNKNOWN_2": 0x0E}
    return self.packer.make_can_msg_safety("ACC_BUTTON", 2, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_STATUS": enable}
    return self.packer.make_can_msg_safety("ACC_2", 2, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_%s" % s: speed for s in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("ABS_1", 0, values)

  def _user_gas_msg(self, gas):
   values = {"ACCEL_PEDAL_USER": gas}
   return self.packer.make_can_msg_safety("ENGINE_2", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ABS_3", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND", 0, values)

  def _torque_cmd_2_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND_2", 0, values)

  def test_lka_command_2_mirrors_lka_command(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._rx(self._torque_driver_msg(0))
    for torque in (0, 3, 6, 3, 0, -3, -6):
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
      # exact 4x copy is allowed
      self.assertTrue(self._tx(self._torque_cmd_2_msg(torque * 4)))
      # anything else is blocked: different torque, or mismatched LKA_ACTIVE
      self.assertFalse(self._tx(self._torque_cmd_2_msg(torque * 4 + 4)))
      self.assertFalse(self._tx(self._torque_cmd_2_msg(torque * 4 + 1)))
      self.assertFalse(self._tx(self._torque_cmd_2_msg(torque * 4, steer_req=0)))

  def test_lka_command_2_zero_after_blocked_lka_command(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._rx(self._torque_driver_msg(0))
    self.assertTrue(self._tx(self._torque_cmd_msg(3)))
    # rate limit violation blocks LKA_COMMAND; LKA_COMMAND_2 can't carry that torque or the last allowed one
    self.assertFalse(self._tx(self._torque_cmd_msg(3 + self.MAX_RATE_UP + 1)))
    self.assertFalse(self._tx(self._torque_cmd_2_msg((3 + self.MAX_RATE_UP + 1) * 4)))
    self.assertFalse(self._tx(self._torque_cmd_2_msg(3 * 4)))
    self.assertTrue(self._tx(self._torque_cmd_2_msg(0, steer_req=0)))

  def test_lka_command_2_without_controls(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_2_msg(100)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))
    self.assertTrue(self._tx(self._torque_cmd_2_msg(0, steer_req=0)))
    self.assertFalse(self._tx(self._torque_cmd_2_msg(4, steer_req=0)))

  WINDDOWN_MAX_US = 1500000
  WINDDOWN_MAX_GAP_US = 50000

  def _engage_with_torque(self, torque):
    self.safety.set_controls_allowed(True)
    self._rx(self._torque_driver_msg(0))
    self._set_prev_torque(torque)
    self.assertTrue(self._tx(self._torque_cmd_msg(torque)))

  def test_winddown_after_disengage(self):
    for sign in (1, -1):
      self.safety.init_tests()
      self._engage_with_torque(sign * 30)
      self.safety.set_controls_allowed(False)
      # magnitude may hold or shrink, never grow
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * 30)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * 31)))
      for torque in range(27, 0, -3):
        self.assertTrue(self._tx(self._torque_cmd_msg(sign * torque)))
      # winddown ends at zero
      self.assertTrue(self._tx(self._torque_cmd_msg(0)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * 3)))

  def test_winddown_limits(self):
    # no sign change, and LKA_ACTIVE must stay set
    self._engage_with_torque(30)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(-3)))
    self.assertFalse(self._tx(self._torque_cmd_msg(27, steer_req=0)))
    # blocked frames don't end the winddown (openpilot may send a frame or two before it sees the brake)
    self.assertTrue(self._tx(self._torque_cmd_msg(27)))

  def test_winddown_time_limit(self):
    for elapsed in (self.WINDDOWN_MAX_US, self.WINDDOWN_MAX_US + 1):
      self.safety.init_tests()
      self._engage_with_torque(300)
      self.safety.set_controls_allowed(False)
      # keep frames coming within the gap limit
      t = 0
      while t + self.WINDDOWN_MAX_GAP_US < elapsed:
        t += self.WINDDOWN_MAX_GAP_US
        self.safety.set_timer(t)
        self.assertTrue(self._tx(self._torque_cmd_msg(300)))
      self.safety.set_timer(elapsed)
      self.assertEqual(elapsed <= self.WINDDOWN_MAX_US, self._tx(self._torque_cmd_msg(300)))

  def test_winddown_gap_limit(self):
    for gap in (self.WINDDOWN_MAX_GAP_US, self.WINDDOWN_MAX_GAP_US + 1):
      self.safety.init_tests()
      self._engage_with_torque(30)
      self.safety.set_controls_allowed(False)
      self.safety.set_timer(gap // 2)
      # a blocked frame doesn't restart the gap timer
      self.assertFalse(self._tx(self._torque_cmd_msg(33)))
      self.safety.set_timer(gap)
      self.assertEqual(gap <= self.WINDDOWN_MAX_GAP_US, self._tx(self._torque_cmd_msg(27)))

  def test_no_winddown_after_violation(self):
    self._engage_with_torque(30)
    # rate limit violation while engaged
    self.assertFalse(self._tx(self._torque_cmd_msg(30 + self.MAX_RATE_UP + 1)))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(27)))

  def test_no_winddown_without_prior_torque(self):
    self._engage_with_torque(0)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(3)))

  def test_winddown_on_brake(self):
    self._rx(self._speed_msg(100))
    self._rx(self._user_brake_msg(False))
    self._engage_with_torque(30)
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._torque_cmd_msg(33)))
    for torque in range(30, -1, -3):
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
      self.assertTrue(self._tx(self._torque_cmd_2_msg(torque * 4)))
      self.assertFalse(self._tx(self._torque_cmd_2_msg(torque * 4 + 4)))

  def test_reengage_during_winddown(self):
    self._engage_with_torque(30)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._torque_cmd_msg(27)))
    self.safety.set_controls_allowed(True)
    # rate limits continue from the last winddown frame
    self.assertTrue(self._tx(self._torque_cmd_msg(27 + self.MAX_RATE_UP)))
    self.assertFalse(self._tx(self._torque_cmd_msg(30 + self.MAX_RATE_UP + 1)))

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # only cancel is allowed, regardless of controls allowed
      self.assertTrue(self._tx(self._button_msg(cancel=True)))
      self.assertFalse(self._tx(self._button_msg(resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=False, resume=False)))

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_driver_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")


if __name__ == "__main__":
  unittest.main()
