#!/usr/bin/env python3
import unittest
from opendbc.safety import libopendbc_py


class TestCanIgnition(unittest.TestCase):
  """Test CAN ignition detection for specific brands"""

  BRANDS_WITH_CAN_IGNITION = ["gm", "mazda", "rivian", "tesla"]

  def setUp(self):
    self.safety = libopendbc_py.Safety()

  def test_gm_ignition_on(self):
    """Test GM ignition on detection"""
    self.safety.set_safety_mode(libopendbc_py.SAFETY_GM)
    # GM ignition on message (0x120)
    msg = [0x00] * 8
    self.safety.safety_rx_hook(libopendbc_py.make_CANPacket(0x120, 0, msg))
    self.assertTrue(self.safety.get_ignition_can())

  def test_gm_ignition_off(self):
    """Test GM ignition off detection"""
    self.safety.set_safety_mode(libopendbc_py.SAFETY_GM)
    # GM ignition off message (0x1F9)
    msg = [0x00] * 8
    self.safety.safety_rx_hook(libopendbc_py.make_CANPacket(0x1F9, 0, msg))
    self.assertFalse(self.safety.get_ignition_can())

  def test_mazda_ignition_on(self):
    """Test Mazda ignition on detection"""
    self.safety.set_safety_mode(libopendbc_py.SAFETY_MAZDA)
    # Mazda ignition on message (0x3A0)
    msg = [0x00] * 8
    self.safety.safety_rx_hook(libopendbc_py.make_CANPacket(0x3A0, 0, msg))
    self.assertTrue(self.safety.get_ignition_can())

  def test_mazda_ignition_off(self):
    """Test Mazda ignition off detection"""
    self.safety.set_safety_mode(libopendbc_py.SAFETY_MAZDA)
    # Mazda ignition off message (0x4A1)
    msg = [0x00] * 8
    self.safety.safety_rx_hook(libopendbc_py.make_CANPacket(0x4A1, 0, msg))
    self.assertFalse(self.safety.get_ignition_can())

  def test_rivian_ignition_on(self):
    """Test Rivian ignition on detection"""
    self.safety.set_safety_mode(libopendbc_py.SAFETY_RIVIAN)
    # Rivian ignition on message (0x292)
    msg = [0x00] * 8
    self.safety.safety_rx_hook(libopendbc_py.make_CANPacket(0x292, 0, msg))
    self.assertTrue(self.safety.get_ignition_can())

  def test_rivian_ignition_off(self):
    """Test Rivian ignition off detection"""
    self.safety.set_safety_mode(libopendbc_py.SAFETY_RIVIAN)
    # Rivian ignition off message (0x3E0)
    msg = [0x00] * 8
    self.safety.safety_rx_hook(libopendbc_py.make_CANPacket(0x3E0, 0, msg))
    self.assertFalse(self.safety.get_ignition_can())

  def test_tesla_ignition_on(self):
    """Test Tesla ignition on detection"""
    self.safety.set_safety_mode(libopendbc_py.SAFETY_TESLA)
    # Tesla ignition on message (0x348)
    msg = [0x00] * 8
    self.safety.safety_rx_hook(libopendbc_py.make_CANPacket(0x348, 0, msg))
    self.assertTrue(self.safety.get_ignition_can())

  def test_other_brands_no_false_positives(self):
    """Ensure CAN ignition doesn't incorrectly trigger for other brands"""
    # Test a few brands that shouldn't have CAN ignition
    test_brands = [
      (libopendbc_py.SAFETY_HONDA, "honda"),
      (libopendbc_py.SAFETY_TOYOTA, "toyota"),
      (libopendbc_py.SAFETY_HYUNDAI, "hyundai"),
    ]

    for safety_mode, brand in test_brands:
      with self.subTest(brand=brand):
        self.safety.set_safety_mode(safety_mode)
        # Send a message that might trigger false positives
        msg = [0x00] * 8
        for addr in [0x120, 0x1F9, 0x3A0, 0x4A1, 0x292, 0x3E0, 0x348]:
          self.safety.safety_rx_hook(libopendbc_py.make_CANPacket(addr, 0, msg))
          self.assertFalse(self.safety.get_ignition_can(),
                          f"{brand} should not detect CAN ignition for addr {hex(addr)}")


if __name__ == "__main__":
  unittest.main()
