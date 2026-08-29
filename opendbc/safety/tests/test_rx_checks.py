#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import make_msg

# Toyota is a mode with checksums, counters and a quality flag, so it exercises every RX check.
# PCM_CRUISE_SM ignores all three, which keeps these tests about the generic machinery.
CHECK_INDEX = 3
ADDR, BUS, LENGTH, FREQUENCY = 0x226, 0, 8, 40
UNKNOWN_ADDR = 0x123


class TestRxChecks(unittest.TestCase):
  """Covers the generic RX check machinery in safety.h, including its misconfiguration guards."""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, 0)
    self.safety.init_tests()

  def tearDown(self):
    # the RX check tables are static, so undo anything a test changed
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, 0)
    self.safety.set_rx_check_frequency(CHECK_INDEX, FREQUENCY)
    self.safety.set_rx_check_ignore_counter(CHECK_INDEX, True)
    self.safety.set_rx_check_max_counter(CHECK_INDEX, 0)

  def test_unknown_address(self):
    # a message we don't check is passed through without touching any RX check state
    self.assertTrue(self.safety.safety_rx_hook(make_msg(BUS, UNKNOWN_ADDR, LENGTH)))
    self.assertTrue(self.safety.safety_rx_hook(make_msg(BUS + 1, ADDR, LENGTH)))

  def test_wrong_length(self):
    # once we've locked onto a message, the same address at another length is not the one we check
    self.assertTrue(self.safety.safety_rx_hook(make_msg(BUS, ADDR, LENGTH)))
    for length in (4, 6, 64):
      self.assertTrue(self.safety.safety_rx_hook(make_msg(BUS, ADDR, length)))

  def test_no_safety_config(self):
    # main ticks before a safety mode is set
    self.safety.safety_tick_no_safety_config()

  def test_lagging(self):
    # controls are dropped when a checked message stops arriving
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(int(2e6))
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_valid_and_on_time(self):
    # a message that just arrived at its expected rate is neither lagging nor invalid
    self.assertTrue(self.safety.safety_rx_hook(make_msg(BUS, ADDR, LENGTH)))
    self.safety.safety_tick_current_safety_config()

  def test_unknown_safety_mode(self):
    # an unknown mode leaves the previous config in place
    self.assertEqual(-1, self.safety.set_safety_hooks(0xFFFF, 0))

  def test_frequency_too_low(self):
    # a message we check has to be fast enough to disengage on in time
    self.safety.set_rx_check_frequency(CHECK_INDEX, 5)
    self.safety.set_controls_allowed(True)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_config_valid())

  def test_counter_without_a_counter_signal(self):
    # a message that wants its counter checked but doesn't declare one is never valid
    self.safety.set_rx_check_max_counter(CHECK_INDEX, 0)
    self.safety.set_rx_check_ignore_counter(CHECK_INDEX, False)
    self.assertFalse(self.safety.safety_rx_hook(make_msg(BUS, ADDR, LENGTH)))
    self.assertFalse(self.safety.safety_config_valid())


if __name__ == "__main__":
  unittest.main()
