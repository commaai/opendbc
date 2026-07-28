#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common

class TestSafety(common.SafetyTestBase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def test_setters_getters(self):
    for setter, getter, first_value, second_value in [
        (self.safety.set_ignition_can, self.safety.get_ignition_can, False, True),
        (self.safety.set_alternative_experience, self.safety.get_alternative_experience, 0, 1),
        (self.safety.set_cruise_engaged_prev, self.safety.get_cruise_engaged_prev, False, True),
        (self.safety.set_desired_angle_last, self.safety.get_desired_angle_last, 2, 5),
        (self.safety.set_honda_bosch_long, self.safety.get_honda_bosch_long, False, True),
        (self.safety.set_acc_main_on, self.safety.get_acc_main_on, False, True),
        (self.safety.set_honda_hw, self.safety.get_honda_hw, False, True)
    ]:
      setter(first_value)
      self.assertEqual(first_value, getter())
      setter(second_value)
      self.assertEqual(second_value, getter())

  def test_setters_getters_pair(self):
    for setter, getter1, getter2, first, second, third, fourth in [
        (self.safety.set_torque_driver, self.safety.get_torque_driver_min, self.safety.get_torque_driver_max, 0, 1, 2, 3),
        (self.safety.set_angle_meas, self.safety.get_angle_meas_min, self.safety.get_angle_meas_max, 0, 1, 2, 3),
        (self.safety.set_curvature_meas, self.safety.get_curvature_meas_min, self.safety.get_curvature_meas_max, 0, 1, 2, 3)
    ]:
      setter(first, second)
      self.assertEqual(first, getter1())
      self.assertEqual(second, getter2())
      setter(third, fourth)
      self.assertEqual(third, getter1())
      self.assertEqual(fourth, getter2())

  def test_set_safety_hooks_invalid_mode(self):
    self.assertEqual(-1, self.safety.set_safety_hooks(65000, 0))


class TestConfigValid(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def test_invalid_config_no_rx(self):
    self.safety._test_setup_safety_config_valid_checks(-1)
    self.assertFalse(self.safety.safety_config_valid())

  def test_valid_config(self):
    self.safety._test_setup_safety_config_valid_checks(-2)
    self.assertTrue(self.safety.safety_config_valid())

  def test_invalid_rx_check(self):
    for i in range(5):
      self.safety._test_setup_safety_config_valid_checks(i)
      self.assertFalse(self.safety.safety_config_valid())

  def test_no_config_safety_tick(self):
    # Shouldn't segfault
    self.safety.safety_tick(libsafety_py.ffi.NULL)

  def test_safety_tick_rx_check_conditions(self):
    # Exercise each term in `lagging || frequency_invalid || !is_msg_valid`.
    test_cases = (
      # frequency_invalid, msg_invalid, timestamp, controls_allowed after tick
      (False, False, 1_000_001, False),  # lagging
      (True, False, 0, False),  # frequency_invalid
      (False, True,  0, False),  # !is_msg_valid
      (False, False, 0, True),   # all terms false
    )
    for frequency_invalid, msg_invalid, timestamp, expected_controls_allowed in test_cases:
      self.safety._test_setup_safety_tick_rx_check(frequency_invalid, msg_invalid)
      self.safety.set_timer(timestamp)
      self.safety.set_controls_allowed(True)
      self.safety.safety_tick_current_safety_config()
      self.assertEqual(expected_controls_allowed, self.safety.get_controls_allowed())

class TestSafetyExtra(common.SafetyTestBase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def test_rx_compute_no_get_checksum(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaNidec, 0)
    self.safety._test_nullify_compute_checksum()
    self.assertFalse(self._rx(common.make_msg(0, 0x158)))

  def test_rx_skip_null_init(self):
    self.safety._test_nullify_init()
    self.safety.set_safety_hooks(65001, 0)
    self.assertEqual(0, self.safety._test_get_rx_checks_len())

  def test_rx_test_counter(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaNidec, 0)
    self.safety._test_setup_test_get_counter()
    self.assertEqual(0, self.safety._test_get_test_wrong_counters())
    self._rx(common.make_msg(0, 1, length = 0))
    self.assertEqual(5, self.safety._test_get_test_wrong_counters())

  def test_rx_check_index_invalid_length(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaNidec, 0)
    self.assertFalse(self._rx(common.make_msg(0, 0x158)))

if __name__ == "__main__":
  unittest.main()
