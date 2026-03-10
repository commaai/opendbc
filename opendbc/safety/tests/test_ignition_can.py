#!/usr/bin/env python3
import unittest
from opendbc.safety.tests.common import make_msg, libsafety_py as libsafety

class TestIgnitionCan(unittest.TestCase):
  def setUp(self):
    libsafety.libsafety.init_tests()
    # Reset ignition state before each test
    libsafety.libsafety.set_safety_hooks(0, 0)  # DEFAULT safety mode

  def test_gm_ignition(self):
    # GM: 0x1F1, bus 0, len 8, data[0] bit 1 set = ignition on
    msg = make_msg(0, 0x1F1, 8, b'\x02\x00\x00\x00\x00\x00\x00\x00')
    libsafety.libsafety.safety_rx_hook(msg)
    self.assertTrue(libsafety.libsafety.get_ignition_can())
    self.assertEqual(libsafety.libsafety.get_ignition_can_cnt(), 0)

    # Test ignition off
    msg = make_msg(0, 0x1F1, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00')
    libsafety.libsafety.safety_rx_hook(msg)
    self.assertFalse(libsafety.libsafety.get_ignition_can())

  def test_rivian_ignition(self):
    # Rivian: 0x152, bus 0, len 8, counter increments correctly, data[7] high 4 bits = 1
    # First msg to set counter
    msg1 = make_msg(0, 0x152, 8, b'\x00\x00\x00\x00\x00\x00\x00\x10')
    libsafety.libsafety.safety_rx_hook(msg1)
    # Second msg with counter +1
    msg2 = make_msg(0, 0x152, 8, b'\x00\x01\x00\x00\x00\x00\x00\x10')
    libsafety.libsafety.safety_rx_hook(msg2)
    self.assertTrue(libsafety.libsafety.get_ignition_can())

    # Test power mode off
    msg3 = make_msg(0, 0x152, 8, b'\x00\x02\x00\x00\x00\x00\x00\x00')
    libsafety.libsafety.safety_rx_hook(msg3)
    self.assertFalse(libsafety.libsafety.get_ignition_can())

  def test_tesla_ignition(self):
    # Tesla: 0x221, bus 0, len 8, counter increments correctly, data[0] high 3 bits = 3
    # First msg to set counter
    msg1 = make_msg(0, 0x221, 8, b'\xE0\x00\x00\x00\x00\x00\x00\x00')  # data[0] = 0xE0 = 0b11100000, high 3 bits 3
    libsafety.libsafety.safety_rx_hook(msg1)
    # Second msg with counter +1 (data[6] high 4 bits = 1)
    msg2 = make_msg(0, 0x221, 8, b'\xE0\x00\x00\x00\x00\x00\x10\x00')
    libsafety.libsafety.safety_rx_hook(msg2)
    self.assertTrue(libsafety.libsafety.get_ignition_can())

    # Test power mode off
    msg3 = make_msg(0, 0x221, 8, b'\x00\x00\x00\x00\x00\x00\x20\x00')
    libsafety.libsafety.safety_rx_hook(msg3)
    self.assertFalse(libsafety.libsafety.get_ignition_can())

  def test_mazda_ignition(self):
    # Mazda: 0x9E, bus 0, len 8, data[0] high 3 bits = 6
    msg = make_msg(0, 0x9E, 8, b'\xC0\x00\x00\x00\x00\x00\x00\x00')  # 0xC0 = 0b11000000, high 3 bits 6
    libsafety.libsafety.safety_rx_hook(msg)
    self.assertTrue(libsafety.libsafety.get_ignition_can())

    # Test ignition off
    msg = make_msg(0, 0x9E, 8, b'\x00\x00\x00\x00\x00\x00\x00\x00')
    libsafety.libsafety.safety_rx_hook(msg)
    self.assertFalse(libsafety.libsafety.get_ignition_can())

  def test_no_false_positives(self):
    # Test random other messages don't trigger ignition
    test_addrs = [0x100, 0x200, 0x300, 0x400, 0x500]
    for addr in test_addrs:
      msg = make_msg(0, addr, 8, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF')
      libsafety.libsafety.safety_rx_hook(msg)
      self.assertFalse(libsafety.libsafety.get_ignition_can())

    # Test bus != 0, even matching addr shouldn't trigger
    msg = make_msg(1, 0x1F1, 8, b'\x02\x00\x00\x00\x00\x00\x00\x00')
    libsafety.libsafety.safety_rx_hook(msg)
    self.assertFalse(libsafety.libsafety.get_ignition_can())

if __name__ == "__main__":
  unittest.main()
