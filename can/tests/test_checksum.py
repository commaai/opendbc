#!/usr/bin/env python3
import unittest
import random

import cereal.messaging as messaging
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from opendbc.can.tests.test_packer_parser import can_list_to_can_capnp

class TestCanChecksum(unittest.TestCase):
  def test_honda_checksum(self):
    """Test checksums for Honda standard and extended CAN ids"""
    dbc_file = "honda_accord_2018_can_generated"

    signals = [("CHECKSUM", "LKAS_HUD"),
              ("COUNTER", "LKAS_HUD"),
              ("CHECKSUM", "LKAS_HUD_A"),
              ("COUNTER", "LKAS_HUD_A")]
    checks = [("LKAS_HUD", 0), ("LKAS_HUD_A", 0)]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    values = {
    'SET_ME_X41': 0x41,
    'STEERING_REQUIRED': 1,
    'SOLID_LANES': 1,
    'BEEP': 0,
    }

    # known correct checksums according to the above values
    checksum_std = [11, 10, 9, 8]
    checksum_ext = [4, 3, 2, 1]
    can_msgs = [[], []]

    idx = 0
    for _ in range(4):
      can_msgs[1].append(packer.make_can_msg("LKAS_HUD", 0, values, idx))
      can_msgs[1].append(packer.make_can_msg("LKAS_HUD_A", 0, values, idx))
      idx += 1

    can_strings = [can_list_to_can_capnp(msgs) for msgs in can_msgs]
    parser.update_strings(can_strings)
    for i, s in enumerate(parser.vl_all["LKAS_HUD"]["CHECKSUM"]):
      self.assertEqual(s, checksum_std[i])
    for ie, se in enumerate(parser.vl_all["LKAS_HUD_A"]["CHECKSUM"]):
      self.assertEqual(se, checksum_ext[ie])

if __name__ == "__main__":
  unittest.main()
