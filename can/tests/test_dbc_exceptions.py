#!/usr/bin/env python3

import unittest

from opendbc.can.parser import CANParser, CANDefine
from opendbc.can.packer import CANPacker

class TestCanParserPackerExceptions(unittest.TestCase):
  def test_civic_exceptions(self):
    dbc_file = "honda_civic_touring_2016_can_generated"
    signals = [
      ("STEER_TORQUE", "STEERING_CONTROL", 0),
      ("STEER_TORQUE_REQUEST", "STEERING_CONTROL", 0),
    ]
    checks = []
    with self.assertRaises(Exception):
      CANParser(dbc_file + "abcdefgh", signals, checks, 0)
    with self.assertRaises(Exception):
      CANPacker(dbc_file + "abcdefgh")
    with self.assertRaises(Exception):
      CANDefine(dbc_file + "abcdefgh")

    # Everything is supposed to work below
    CANParser(dbc_file, signals, checks, 0)
    CANPacker(dbc_file)
    CANDefine(dbc_file)


if __name__ == "__main__":
  unittest.main()
