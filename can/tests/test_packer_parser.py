#!/usr/bin/env python3
from collections import namedtuple
import unittest

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
      ("STEER_TORQUE", "STEERING_CONTROL", 0),
      ("STEER_TORQUE_REQUEST", "STEERING_CONTROL", 0),
    ]
    checks = [("STEERING_CONTROL", 50)]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    idx = 0

    for steer in range(-256, 255):
      for active in [1, 0]:
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

        idx += 1

  def test_scale_offset(self):
    """Test that both scale and offset are correctly preserved"""
    dbc_file = "honda_civic_touring_2016_can_generated"

    signals = [
      ("USER_BRAKE", "VSA_STATUS", 0),
    ]
    checks = [("VSA_STATUS", 50)]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    idx = 0
    for brake in range(101):
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
      ("Counter", "ES_LKAS", 0),
      ("LKAS_Output", "ES_LKAS", 0),
      ("LKAS_Request", "ES_LKAS", 0),
      ("SET_1", "ES_LKAS", 0),
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

  def test_eps_torque_scale(self):
    """Test Toyota EPS torque scaling detection"""
    SignalInfo = namedtuple("SignalInfo", ["dbc", "msg", "sig", "param", "value"])
    signals = [
      SignalInfo("toyota_nodsu_pt_generated", "STEER_TORQUE_SENSOR", "STEER_TORQUE_EPS", "scale", 0.73),
      SignalInfo("toyota_prius_2017_pt_generated", "STEER_TORQUE_SENSOR", "STEER_TORQUE_EPS", "scale", 0.66),
      SignalInfo("toyota_corolla_2017_pt_generated", "STEER_TORQUE_SENSOR", "STEER_TORQUE_EPS", "scale", 0.88),
    ]

    for info in signals:
      parser = CANParser(info.dbc, [])
      self.assertAlmostEqual(parser.info[info.msg][info.sig][info.param], info.value)


if __name__ == "__main__":
  unittest.main()
