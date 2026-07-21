#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

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


if __name__ == "__main__":
  unittest.main()
