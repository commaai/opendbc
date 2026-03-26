#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestChryslerCuswSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x1F6, 0], [0x2FA, 0], [0x5DC, 0]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (0x1F6, 0x5DC)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1F6, 0x5DC]}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 4
  MAX_TORQUE_LOOKUP = [0], [250]
  MAX_RT_DELTA = 150
  MAX_TORQUE_ERROR = 80

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_cusw")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chryslerCusw, 0)
    self.safety.init_tests()

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": 1 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_CONTROL", 0, values)

  def _speed_msg(self, speed):
    values = {"VEHICLE_SPEED": speed}
    return self.packer.make_can_msg_safety("BRAKE_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_HUMAN": gas}
    return self.packer.make_can_msg_safety("ACCEL_GAS", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_HUMAN": 1 if brake else 0}
    return self.packer.make_can_msg_safety("BRAKE_2", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("EPS_STATUS", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": steer_req}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_meas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")

  def _set_prev_torque_with_ramp(self, t):
    """Set all torque tracking state including the CUSW ramp-down tracker."""
    self._set_prev_torque(t)
    self.safety.set_chrysler_cusw_torque_last(t)

  def test_ramp_down_positive_torque(self):
    """After controls disallowed, positive torque must ramp to zero at MAX_RATE_DOWN"""
    # Build up torque while controls allowed
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(100)
    self.assertTrue(self._tx(self._torque_cmd_msg(100)))

    # Disallow controls
    self.safety.set_controls_allowed(False)

    # Must ramp down, can't stay at same value
    self.assertFalse(self._tx(self._torque_cmd_msg(100)))

    # Reset and try valid ramp-down
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(100)
    self.assertTrue(self._tx(self._torque_cmd_msg(100)))
    self.safety.set_controls_allowed(False)

    # Ramp down at MAX_RATE_DOWN per frame
    torque = 100
    while torque > 0:
      new_torque = max(torque - self.MAX_RATE_DOWN, 0)
      steer_req = 1 if new_torque != 0 else 0
      self.assertTrue(self._tx(self._torque_cmd_msg(new_torque, steer_req)), f"ramp-down failed at {torque} -> {new_torque}")
      torque = new_torque

    # After reaching zero, must stay at zero
    self.assertTrue(self._tx(self._torque_cmd_msg(0, 0)))
    self.assertFalse(self._tx(self._torque_cmd_msg(1, 1)))

  def test_ramp_down_negative_torque(self):
    """After controls disallowed, negative torque must ramp to zero at MAX_RATE_DOWN"""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(-100)
    self.assertTrue(self._tx(self._torque_cmd_msg(-100)))

    self.safety.set_controls_allowed(False)

    # Ramp up toward zero at MAX_RATE_DOWN per frame
    torque = -100
    while torque < 0:
      new_torque = min(torque + self.MAX_RATE_DOWN, 0)
      steer_req = 1 if new_torque != 0 else 0
      self.assertTrue(self._tx(self._torque_cmd_msg(new_torque, steer_req)), f"ramp-down failed at {torque} -> {new_torque}")
      torque = new_torque

    self.assertTrue(self._tx(self._torque_cmd_msg(0, 0)))

  def test_ramp_down_too_fast(self):
    """Torque can't decrease faster than MAX_RATE_DOWN during ramp-down"""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(100)
    self.assertTrue(self._tx(self._torque_cmd_msg(100)))

    self.safety.set_controls_allowed(False)

    # Dropping by more than MAX_RATE_DOWN should fail
    self.assertFalse(self._tx(self._torque_cmd_msg(100 - self.MAX_RATE_DOWN - 1)))

  def test_ramp_down_wrong_direction(self):
    """Torque can't increase magnitude during ramp-down"""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(50)
    self.assertTrue(self._tx(self._torque_cmd_msg(50)))

    self.safety.set_controls_allowed(False)

    # Can't increase
    self.assertFalse(self._tx(self._torque_cmd_msg(51)))
    # Can't cross zero
    self.assertFalse(self._tx(self._torque_cmd_msg(-1)))

  def test_ramp_down_steer_req_must_match(self):
    """LKAS_CONTROL_BIT must be set while torque non-zero during ramp-down"""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(20)
    self.assertTrue(self._tx(self._torque_cmd_msg(20)))

    self.safety.set_controls_allowed(False)

    # Non-zero torque with steer_req=0 must fail
    self.assertFalse(self._tx(self._torque_cmd_msg(16, steer_req=0)))

    # Non-zero torque with steer_req=1 must pass (valid ramp-down step)
    self.safety.set_chrysler_cusw_torque_last(20)
    self.assertTrue(self._tx(self._torque_cmd_msg(16, steer_req=1)))

    # Zero torque with either steer_req value is fine
    self.safety.set_chrysler_cusw_torque_last(self.MAX_RATE_DOWN)
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))
    self.safety.set_chrysler_cusw_torque_last(self.MAX_RATE_DOWN)
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=1)))

  def test_ramp_down_violation_resets(self):
    """After a ramp-down violation, must send zero torque"""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(50)
    self.assertTrue(self._tx(self._torque_cmd_msg(50)))

    self.safety.set_controls_allowed(False)

    # Cause a violation (wrong direction)
    self.assertFalse(self._tx(self._torque_cmd_msg(51)))

    # After violation, chrysler_cusw_torque_last is reset to 0,
    # so now only zero torque with steer_req=0 is allowed
    self.assertFalse(self._tx(self._torque_cmd_msg(46, steer_req=1)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))


if __name__ == "__main__":
  unittest.main()
