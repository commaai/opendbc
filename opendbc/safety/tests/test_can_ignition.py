#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import make_msg


class TestCANIgnition(unittest.TestCase):
  # Not a safety mode test, so no TX_MSGS
  TX_MSGS = None

  @classmethod
  def setUpClass(cls):
    cls.safety = libsafety_py.libsafety

  def setUp(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.safety.init_tests()
    self.assertFalse(self.safety.get_ignition_can())

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  # GM - address 0x1F1
  def test_gm_ignition_on(self):
    msg = make_msg(0, 0x1F1, 8, b'\x02\x00\x00\x00\x00\x00\x00\x00')
    self._rx(msg)
    self.assertTrue(self.safety.get_ignition_can())

  def test_gm_ignition_crank(self):
    msg = make_msg(0, 0x1F1, 8, b'\x03\x00\x00\x00\x00\x00\x00\x00')
    self._rx(msg)
    self.assertTrue(self.safety.get_ignition_can())

  def test_gm_ignition_off(self):
    self._rx(make_msg(0, 0x1F1, 8, b'\x02\x00\x00\x00\x00\x00\x00\x00'))
    self.assertTrue(self.safety.get_ignition_can())
    self._rx(make_msg(0, 0x1F1, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  def test_gm_wrong_bus(self):
    self._rx(make_msg(1, 0x1F1, 8, b'\x02\x00\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  def test_gm_wrong_length(self):
    self._rx(make_msg(0, 0x1F1, 4, b'\x02\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  # Mazda - address 0x9E
  def test_mazda_ignition_on(self):
    msg = make_msg(0, 0x9E, 8, b'\xC0\x00\x00\x00\x00\x00\x00\x00')
    self._rx(msg)
    self.assertTrue(self.safety.get_ignition_can())

  def test_mazda_ignition_off(self):
    self._rx(make_msg(0, 0x9E, 8, b'\xC0\x00\x00\x00\x00\x00\x00\x00'))
    self.assertTrue(self.safety.get_ignition_can())
    self._rx(make_msg(0, 0x9E, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  def test_mazda_wrong_bus(self):
    self._rx(make_msg(1, 0x9E, 8, b'\xC0\x00\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  # Tesla - address 0x221, requires counter sequence
  def test_tesla_counter_required(self):
    # first msg establishes counter, doesn't trigger
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())
    # second msg with sequential counter triggers
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\x10\x00'))
    self.assertTrue(self.safety.get_ignition_can())

  def test_tesla_ignition_off(self):
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\x00\x00'))
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\x10\x00'))
    self.assertTrue(self.safety.get_ignition_can())
    self._rx(make_msg(0, 0x221, 8, b'\x00\x00\x00\x00\x00\x00\x20\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  def test_tesla_bad_counter(self):
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())
    # skip to counter 5, not sequential
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\x50\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  def test_tesla_counter_wrap(self):
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\xE0\x00'))  # 14
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\xF0\x00'))  # 15
    self.assertTrue(self.safety.get_ignition_can())
    self._rx(make_msg(0, 0x221, 8, b'\x60\x00\x00\x00\x00\x00\x00\x00'))  # 0
    self.assertTrue(self.safety.get_ignition_can())

  # Rivian - address 0x152, requires counter sequence
  def test_rivian_counter_required(self):
    self._rx(make_msg(0, 0x152, 8, b'\x00\x00\x00\x00\x00\x00\x00\x10'))
    self.assertFalse(self.safety.get_ignition_can())
    self._rx(make_msg(0, 0x152, 8, b'\x00\x01\x00\x00\x00\x00\x00\x10'))
    self.assertTrue(self.safety.get_ignition_can())

  def test_rivian_ignition_off(self):
    self._rx(make_msg(0, 0x152, 8, b'\x00\x00\x00\x00\x00\x00\x00\x10'))
    self._rx(make_msg(0, 0x152, 8, b'\x00\x01\x00\x00\x00\x00\x00\x10'))
    self.assertTrue(self.safety.get_ignition_can())
    self._rx(make_msg(0, 0x152, 8, b'\x00\x02\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  def test_rivian_counter_wrap(self):
    self._rx(make_msg(0, 0x152, 8, b'\x00\x0D\x00\x00\x00\x00\x00\x10'))  # 13
    self._rx(make_msg(0, 0x152, 8, b'\x00\x0E\x00\x00\x00\x00\x00\x10'))  # 14
    self.assertTrue(self.safety.get_ignition_can())
    self._rx(make_msg(0, 0x152, 8, b'\x00\x00\x00\x00\x00\x00\x00\x10'))  # 0
    self.assertTrue(self.safety.get_ignition_can())

  # false positive checks
  def test_no_false_positive_honda(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaBosch, 0)
    self.safety.init_tests()
    self._rx(make_msg(0, 0x1F1, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self._rx(make_msg(0, 0x9E, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self._rx(make_msg(0, 0x152, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self._rx(make_msg(0, 0x221, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  def test_no_false_positive_toyota(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, 0)
    self.safety.init_tests()
    self._rx(make_msg(0, 0x1F1, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self._rx(make_msg(0, 0x9E, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self._rx(make_msg(0, 0x152, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self._rx(make_msg(0, 0x221, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00'))
    self.assertFalse(self.safety.get_ignition_can())

  def test_subaru_no_false_positive_0x152(self):
    # 0x152 overlaps Subaru pre-global high beam
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    for _ in range(10):
      self._rx(make_msg(0, 0x152, 8, b'\x00\x00\x00\x00\x00\x00\x00\x10'))
    self.assertFalse(self.safety.get_ignition_can())

  # state reset
  def test_reset_on_safety_mode_change(self):
    self._rx(make_msg(0, 0x1F1, 8, b'\x02\x00\x00\x00\x00\x00\x00\x00'))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, 0)
    self.assertFalse(self.safety.get_ignition_can())

  def test_set_ignition_can_helper(self):
    self.assertFalse(self.safety.get_ignition_can())
    self.safety.set_ignition_can(True)
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.set_ignition_can(False)
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
