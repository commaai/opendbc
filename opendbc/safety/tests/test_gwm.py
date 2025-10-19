#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerPanda


class TestGwm(common.PandaSafetyTest):
  TX_MSGS = [[0xA1, 0]]

  def setUp(self):
    self.packer = CANPackerPanda("gwm_haval_h6_mk3_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm, 0)
    self.safety.init_tests()

  def test_hello(self):
    print('hello')
    self.assertTrue(True)

if __name__ == "__main__":
  unittest.main()
