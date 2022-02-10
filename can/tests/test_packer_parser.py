#!/usr/bin/env python3
import unittest
import random

import cereal.messaging as messaging
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker

# Python implementation so we don't have to depend on boardd
def can_list_to_can_capnp(can_msgs, msgtype='can'):
  dat = messaging.new_message()
  dat.init(msgtype, len(can_msgs))

  for i, can_msg in enumerate(can_msgs):
    if msgtype == 'sendcan':
      cc = dat.sendcan[i]
    else:
      cc = dat.can[i]

    cc.address = can_msg[0]
    cc.busTime = can_msg[1]
    cc.dat = bytes(can_msg[2])
    cc.src = can_msg[3]

  return dat.to_bytes()


class TestCanParserPacker(unittest.TestCase):
  def test_civic(self):

    dbc_file = "honda_civic_touring_2016_can_generated"

    signals = [
      ("STEER_TORQUE", "STEERING_CONTROL"),
      ("STEER_TORQUE_REQUEST", "STEERING_CONTROL"),
    ]
    checks = [("STEERING_CONTROL", 50)]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    idx = 0

    for steer in range(-256, 255):
      for active in (1, 0):
        values = {
          "STEER_TORQUE": steer,
          "STEER_TORQUE_REQUEST": active,
        }

        msgs = packer.make_can_msg("STEERING_CONTROL", 0, values, idx)
        bts = can_list_to_can_capnp([msgs])

        parser.update_string(bts)

        self.assertAlmostEqual(parser.vl["STEERING_CONTROL"]["STEER_TORQUE"], steer)
        self.assertAlmostEqual(parser.vl["STEERING_CONTROL"]["STEER_TORQUE_REQUEST"], active)
        self.assertAlmostEqual(parser.vl["STEERING_CONTROL"]["COUNTER"], idx % 4)

        for sig in ("STEER_TORQUE", "STEER_TORQUE_REQUEST", "COUNTER", "CHECKSUM"):
          self.assertEqual(parser.vl["STEERING_CONTROL"][sig], parser.vl[228][sig])

        idx += 1

  def test_scale_offset(self):
    """Test that both scale and offset are correctly preserved"""
    dbc_file = "honda_civic_touring_2016_can_generated"

    signals = [
      ("USER_BRAKE", "VSA_STATUS"),
    ]
    checks = [("VSA_STATUS", 50)]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    idx = 0
    for brake in range(0, 100):
      values = {"USER_BRAKE": brake}
      msgs = packer.make_can_msg("VSA_STATUS", 0, values, idx)
      bts = can_list_to_can_capnp([msgs])

      parser.update_string(bts)

      self.assertAlmostEqual(parser.vl["VSA_STATUS"]["USER_BRAKE"], brake)
      idx += 1

  def test_subaru(self):
    # Subuaru is little endian

    dbc_file = "subaru_global_2017_generated"

    signals = [
      ("Counter", "ES_LKAS"),
      ("LKAS_Output", "ES_LKAS"),
      ("LKAS_Request", "ES_LKAS"),
      ("SET_1", "ES_LKAS"),
    ]
    checks = [("ES_LKAS", 50)]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    idx = 0

    for steer in range(-256, 255):
      for active in [1, 0]:
        values = {
          "Counter": idx,
          "LKAS_Output": steer,
          "LKAS_Request": active,
          "SET_1": 1
        }

        msgs = packer.make_can_msg("ES_LKAS", 0, values)
        bts = can_list_to_can_capnp([msgs])
        parser.update_string(bts)

        self.assertAlmostEqual(parser.vl["ES_LKAS"]["LKAS_Output"], steer)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["LKAS_Request"], active)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["SET_1"], 1)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["Counter"], idx % 16)

        idx += 1

  def test_updated(self):
    """Test updated value dict"""
    dbc_file = "honda_civic_touring_2016_can_generated"

    signals = [("USER_BRAKE", "VSA_STATUS")]
    checks = [("VSA_STATUS", 50)]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    # Make sure nothing is updated
    self.assertEqual(len(parser.vl_all["VSA_STATUS"]["USER_BRAKE"]), 0)

    idx = 0
    for _ in range(10):
      # Ensure CANParser holds the values of any duplicate messages over multiple frames
      user_brake_vals = [random.randrange(100) for _ in range(random.randrange(5, 10))]
      half_idx = len(user_brake_vals) // 2
      can_msgs = [[], []]
      for frame, brake_vals in enumerate((user_brake_vals[:half_idx], user_brake_vals[half_idx:])):
        for user_brake in brake_vals:
          values = {"USER_BRAKE": user_brake}
          can_msgs[frame].append(packer.make_can_msg("VSA_STATUS", 0, values, idx))
          idx += 1

      can_strings = [can_list_to_can_capnp(msgs) for msgs in can_msgs]
      parser.update_strings(can_strings)
      vl_all = parser.vl_all["VSA_STATUS"]["USER_BRAKE"]

      self.assertEqual(vl_all, user_brake_vals)
      if len(user_brake_vals):
        self.assertEqual(vl_all[-1], parser.vl["VSA_STATUS"]["USER_BRAKE"])


if __name__ == "__main__":
  unittest.main()
