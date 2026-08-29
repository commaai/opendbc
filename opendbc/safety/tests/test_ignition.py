#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import make_msg

# The brands we detect ignition from, and the length their message has to be.
# Addresses overlap between brands, so a wrong length is a different car's message.
IGNITION_MSGS = {
  "gm": (0x1F1, 8),
  "rivian": (0x152, 8),
  "tesla": (0x221, 8),
  "mazda": (0x9E, 8),
  "volkswagen_meb": (0x3C0, 4),
}


class TestIgnitionCanHook(unittest.TestCase):

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.silent, 0)
    self.safety.init_tests()

  def test_ignored_buses(self):
    # ignition is only read off the powertrain bus
    for bus in range(1, 4):
      for addr, length in IGNITION_MSGS.values():
        self.safety.set_ignition_can(False)
        self.safety.ignition_can_hook(make_msg(bus, addr, length, dat=b'\xff' * length))
        self.assertFalse(self.safety.get_ignition_can(), f"{bus=} {addr=:x}")

  def test_wrong_length(self):
    # another brand's message can share the address, so the length has to match
    for brand, (addr, length) in IGNITION_MSGS.items():
      for wrong_length in (4, 8):
        if wrong_length == length:
          continue
        self.safety.set_ignition_can(False)
        self.safety.ignition_can_hook(make_msg(0, addr, wrong_length, dat=b'\xff' * wrong_length))
        self.assertFalse(self.safety.get_ignition_can(), f"{brand} {addr=:x} {wrong_length=}")


if __name__ == "__main__":
  unittest.main()
