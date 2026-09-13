#!/usr/bin/env python3
import unittest
import itertools

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class TestDefaultRxHookBase(common.SafetyTest):
  FWD_BUS_LOOKUP = {}

  def test_rx_hook(self):
    # default rx hook allows all msgs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)), f"failed RX {addr=}")


class TestNoOutput(TestDefaultRxHookBase):
  TX_MSGS = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.safety.init_tests()


class TestSilent(TestNoOutput):
  """SILENT uses same hooks as NOOUTPUT"""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.silent, 0)
    self.safety.init_tests()


class TestAllOutput(TestDefaultRxHookBase):
  # Allow all messages
  TX_MSGS = [[addr, bus] for addr in common.SafetyTest.SCANNED_ADDRS
             for bus in range(4)]

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.allOutput, 0)
    self.safety.init_tests()

  def test_spam_can_buses(self):
    # asserts tx allowed for all scanned addrs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        should_tx = [addr, bus] in self.TX_MSGS
        self.assertEqual(should_tx, self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  def test_default_controls_not_allowed(self):
    # controls always allowed
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_hook_on_wrong_safety_mode(self):
    # No point, since we allow all messages
    pass


class TestAllOutputPassthrough(TestAllOutput):
  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.allOutput, 1)
    self.safety.init_tests()


class TestSafetyFramework(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.safety.init_tests()

  def test_unsupported_safety_mode(self):
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.safety.set_safety_hooks(0xFFFF, 0), -1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(common.make_msg(0, 0x123)))

  def test_rx_callbacks_required(self):
    # The fixture has no counter callback, so counters must be explicitly ignored.
    msg = common.make_msg(0, 0x123)
    for getter, computer, ignore_checksum, ignore_counter in itertools.product((False, True), repeat=4):
      with self.subTest(getter=getter, computer=computer, ignore_checksum=ignore_checksum, ignore_counter=ignore_counter):
        self.safety.set_controls_allowed(True)
        valid = (ignore_checksum or (getter and computer)) and ignore_counter
        self.assertEqual(self.safety.rx_check_callbacks(msg, getter, computer, ignore_checksum, ignore_counter), valid)
        self.assertEqual(self.safety.get_controls_allowed(), valid)

  def test_watchdog_faults(self):
    for frequency, timeout in ((5, 2000000), (10, 1000000), (100, 1000000)):
      for elapsed, checksum, quality, wrong_counters in itertools.product((0, timeout, timeout + 1), (False, True), (False, True), (0, 5)):
        with self.subTest(frequency=frequency, elapsed=elapsed, checksum=checksum, quality=quality, wrong_counters=wrong_counters):
          self.safety.set_timer(elapsed)
          self.safety.set_controls_allowed(True)
          lagging = elapsed > timeout
          valid = not lagging and frequency >= 10 and checksum and quality and wrong_counters < 5
          self.assertEqual(self.safety.safety_tick_rx_check(frequency, 0, checksum, quality, wrong_counters), lagging)
          self.assertEqual(self.safety.get_safety_rx_checks_invalid(), not valid)
          self.assertEqual(self.safety.get_controls_allowed(), valid)

  def test_watchdog_timer_wraparound(self):
    self.safety.set_timer(100)
    for elapsed in (300, 1000001):
      self.safety.set_controls_allowed(True)
      last_timestamp = (100 - elapsed) & 0xFFFFFFFF
      lagging = elapsed > 1000000
      self.assertEqual(self.safety.safety_tick_rx_check(100, last_timestamp, True, True, 0), lagging)
      self.assertEqual(self.safety.get_controls_allowed(), not lagging)


if __name__ == "__main__":
  unittest.main()
