#!/usr/bin/env python3
import os
import re
import subprocess
import sys
import unittest
from unittest import mock

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

  def test_compute_mismatch(self):
    self.safety.safety_compute_inject()
    self.assertTrue(self.safety.get_safety_compute_mismatch())
    self.safety.init_tests()

  @unittest.skipIf(sys.platform == "darwin", "reads ELF sections")
  def test_safety_state_tagged(self):
    # writable globals outside the safety_state section aren't computed twice
    not_state = {"ignition_can", "ignition_can_cnt", "prev_counter_rivian", "prev_counter_tesla", "prev_counter_vw_meb",
                 "safety_compute_mismatch", "safety_state_in", "safety_state_out", "safety_ts", "timer_cnt"}
    # without the coverage runtime's globals
    with mock.patch.dict(os.environ, {"SAFETY_COVERAGE": "0"}):
      lib = libsafety_py._build_libsafety()
    out = subprocess.check_output(["nm", "-f", "sysv", "--defined-only", lib], text=True)
    names = set()
    for line in out.splitlines():
      fields = [f.strip() for f in line.split("|")]
      if len(fields) == 7 and fields[6] in (".data", ".bss") and not fields[0].startswith("_") and fields[0] != "completed.0":
        names.add(re.sub(r"\.\d+$", "", fields[0]))
    self.assertEqual(names - not_state, set())


if __name__ == "__main__":
  unittest.main()
