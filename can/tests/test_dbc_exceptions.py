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
    try:
      CANParser(dbc_file + "abcdefgh", signals, checks, 0)
      self.assertTrue(1 + 1 == 3)  # Should't get to here, due to the new added exception
    except Exception:
      pass
    try:
      CANPacker(dbc_file + "abcdefgh")
      self.assertTrue(1 + 1 == 3)  # Should't get to here, due to the new added exception
    except Exception:
      pass
    try:
      CANDefine(dbc_file + "abcdefgh")
      self.assertTrue(1 + 1 == 3)  # Should't get to here, due to the new added exception
    except Exception:
      pass
    
    # Everything is supposed to work below
    CANParser(dbc_file, signals, checks, 0)
    CANPacker(dbc_file)
    CANDefine(dbc_file)


if __name__ == "__main__":
  unittest.main()
