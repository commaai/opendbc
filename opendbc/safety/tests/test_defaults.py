#!/usr/bin/env python3
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams


class TestDefaultRxHookBase(common.SafetyTest):
  FWD_BUS_LOOKUP = {}

  def test_rx_hook(self):
    # default rx hook allows all msgs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)), f"failed RX {addr=}")


class TestNoOutput(TestDefaultRxHookBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  TX_MSGS = []


class TestSilent(TestNoOutput):
  """SILENT uses same hooks as NOOUTPUT"""
  SAFETY_MODEL = CarParams.SafetyModel.silent


class TestAllOutput(TestDefaultRxHookBase):
  SAFETY_MODEL = CarParams.SafetyModel.allOutput

  # Allow all messages
  TX_MSGS = [[addr, bus] for addr in common.SafetyTest.SCANNED_ADDRS
             for bus in range(4)]

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
  SAFETY_PARAM = 1

  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}


class TestSafetyFramework(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  def test_unsupported_safety_mode(self):
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.safety.set_safety_hooks(0xFFFF, 0), -1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(common.make_msg(0, 0x123)))

  # these tests set the flag on purpose
  def tearDown(self):
    self.safety.init_tests()

  def test_controls_allowed_mismatch(self):
    for hook in (self._tx, self._rx):
      with self.subTest(hook=hook.__name__):
        self.safety.init_tests()
        self.safety.set_controls_allowed(True)
        hook(common.make_msg(0, 0x123))
        self.assertFalse(self.safety.get_safety_state_invalid())

        self.safety.set_controls_allowed_inv(True)
        hook(common.make_msg(0, 0x123))
        self.assertTrue(self.safety.get_safety_state_invalid())

  def test_controls_allowed_mismatch_tick(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_inv(True)
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_safety_state_invalid())

  def test_clear_controls_allowed(self):
    self.safety.set_controls_allowed(True)
    self.safety.clear_controls_allowed()
    self._tx(common.make_msg(0, 0x123))
    self.assertFalse(self.safety.get_safety_state_invalid())
    self.assertFalse(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
