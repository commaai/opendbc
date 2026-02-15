#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.libsafety import libsafety_py


class TestIgnitionCan(unittest.TestCase):
  # Required by SafetyTest.test_tx_hook_on_wrong_safety_mode, which imports all
  # test_*.py modules and expects any `Test*` class to define TX_MSGS.
  TX_MSGS = None

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.ignition_can_reset()

  def _rx_ign(self, addr: int, bus: int, data):
    msg = libsafety_py.make_CANPacket(addr, bus, data)
    self.safety.ignition_can_hook(msg)
    return msg

  def test_reset(self):
    self.safety.set_ignition_can(True)
    self.safety.set_ignition_can_cnt(123)
    self.safety.ignition_can_reset()
    self.assertFalse(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

  def test_bus_not_zero_noop(self):
    self.safety.set_ignition_can(True)
    self.safety.set_ignition_can_cnt(123)
    self._rx_ign(0x1F1, 1, [0x0] * 8)
    self.assertTrue(self.safety.get_ignition_can())
    self.assertEqual(123, self.safety.get_ignition_can_cnt())

  def test_gm_1f1(self):
    self.safety.set_ignition_can(False)
    self.safety.set_ignition_can_cnt(5)
    self._rx_ign(0x1F1, 0, [0x2] + ([0x0] * 7))
    self.assertTrue(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

    self.safety.set_ignition_can_cnt(7)
    self._rx_ign(0x1F1, 0, [0x0] * 8)
    self.assertFalse(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

  def test_mazda_9e(self):
    # (data[0] >> 5) == 0x6
    self.safety.set_ignition_can(False)
    self.safety.set_ignition_can_cnt(9)
    self._rx_ign(0x9E, 0, [0xC0] + ([0x0] * 7))
    self.assertTrue(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

    self.safety.set_ignition_can_cnt(11)
    self._rx_ign(0x9E, 0, [0x00] * 8)
    self.assertFalse(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

  def test_rivian_152_sequential_counter_sets(self):
    self.safety.set_ignition_can(False)
    self._rx_ign(0x152, 0, [0x0, 0x0] + ([0x0] * 5) + [0x10])
    self.assertFalse(self.safety.get_ignition_can())

    self.safety.set_ignition_can_cnt(42)
    self._rx_ign(0x152, 0, [0x0, 0x1] + ([0x0] * 5) + [0x10])
    self.assertTrue(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

    self.safety.set_ignition_can_cnt(7)
    self._rx_ign(0x152, 0, [0x0, 0x2] + ([0x0] * 5) + [0x00])
    self.assertFalse(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

  def test_rivian_152_skipped_counter_no_update(self):
    self.safety.ignition_can_reset()
    self.safety.set_ignition_can(False)
    self._rx_ign(0x152, 0, [0x0, 0x0] + ([0x0] * 5) + [0x10])
    self.assertFalse(self.safety.get_ignition_can())

    # Skip counter, should not update ignition state
    self.safety.set_ignition_can_cnt(99)
    self._rx_ign(0x152, 0, [0x0, 0x2] + ([0x0] * 5) + [0x10])
    self.assertFalse(self.safety.get_ignition_can())
    self.assertEqual(99, self.safety.get_ignition_can_cnt())

  def test_tesla_221_sequential_counter_sets(self):
    self.safety.set_ignition_can(False)
    # First message initializes counter state, should not update
    self._rx_ign(0x221, 0, [0x60] + ([0x0] * 5) + [0x00, 0x00])
    self.assertFalse(self.safety.get_ignition_can())

    self.safety.set_ignition_can_cnt(8)
    # power_state == 3 (bits 6..5 == 0b11)
    self._rx_ign(0x221, 0, [0x60] + ([0x0] * 5) + [0x10, 0x00])
    self.assertTrue(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

    self.safety.set_ignition_can_cnt(12)
    # power_state != 3, update to false
    self._rx_ign(0x221, 0, [0x00] + ([0x0] * 5) + [0x20, 0x00])
    self.assertFalse(self.safety.get_ignition_can())
    self.assertEqual(0, self.safety.get_ignition_can_cnt())

  def test_tesla_221_skipped_counter_no_update(self):
    self.safety.ignition_can_reset()
    self.safety.set_ignition_can(False)
    self._rx_ign(0x221, 0, [0x60] + ([0x0] * 5) + [0x00, 0x00])
    self.assertFalse(self.safety.get_ignition_can())

    self.safety.set_ignition_can_cnt(77)
    # Skip counter, should not update ignition state
    self._rx_ign(0x221, 0, [0x60] + ([0x0] * 5) + [0x30, 0x00])
    self.assertFalse(self.safety.get_ignition_can())
    self.assertEqual(77, self.safety.get_ignition_can_cnt())


if __name__ == "__main__":
  unittest.main()
