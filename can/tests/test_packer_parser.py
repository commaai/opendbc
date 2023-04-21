#!/usr/bin/env python3
import os
import unittest
import random

import cereal.messaging as messaging
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker


TEST_DBC = os.path.abspath(os.path.join(os.path.dirname(__file__), "test.dbc"))


# Python implementation so we don't have to depend on boardd
def can_list_to_can_capnp(can_msgs, msgtype='can', logMonoTime=None):
  dat = messaging.new_message()
  dat.init(msgtype, len(can_msgs))

  if logMonoTime is not None:
    dat.logMonoTime = logMonoTime

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
  def test_packer(self):
    packer = CANPacker(TEST_DBC)

    for b in range(6):
      for i in range(256):
        values = {"COUNTER": i}
        addr, _, dat, bus = packer.make_can_msg("CAN_FD_MESSAGE", b, values)
        self.assertEqual(addr, 245)
        self.assertEqual(bus, b)
        self.assertEqual(dat[0], i)

  def test_packer_counter(self):
    signals = [
      ("COUNTER", "CAN_FD_MESSAGE"),
    ]
    checks = [("CAN_FD_MESSAGE", 0), ]
    packer = CANPacker(TEST_DBC)
    parser = CANParser(TEST_DBC, signals, checks, 0)

    # packer should increment the counter
    for i in range(1000):
      msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {})
      dat = can_list_to_can_capnp([msg, ])
      parser.update_strings([dat])
      self.assertEqual(parser.vl["CAN_FD_MESSAGE"]["COUNTER"], i % 256)

    # setting COUNTER should override
    for _ in range(100):
      cnt = random.randint(0, 255)
      msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {
        "COUNTER": cnt,
      })
      dat = can_list_to_can_capnp([msg, ])
      parser.update_strings([dat])
      self.assertEqual(parser.vl["CAN_FD_MESSAGE"]["COUNTER"], cnt)

    # then, should resume counting from the override value
    cnt = parser.vl["CAN_FD_MESSAGE"]["COUNTER"]
    for i in range(100):
      msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {})
      dat = can_list_to_can_capnp([msg, ])
      parser.update_strings([dat])
      self.assertEqual(parser.vl["CAN_FD_MESSAGE"]["COUNTER"], (cnt + i) % 256)

  def test_parser_can_valid(self):
    signals = [
      ("COUNTER", "CAN_FD_MESSAGE"),
    ]
    checks = [("CAN_FD_MESSAGE", 10), ]
    packer = CANPacker(TEST_DBC)
    parser = CANParser(TEST_DBC, signals, checks, 0)

    # shouldn't be valid initially
    self.assertFalse(parser.can_valid)

    # not valid until the message is seen
    for _ in range(100):
      dat = can_list_to_can_capnp([])
      parser.update_strings([dat])
      self.assertFalse(parser.can_valid)

    # valid once seen
    for i in range(1, 100):
      t = int(0.01 * i * 1e9)
      msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {})
      dat = can_list_to_can_capnp([msg, ], logMonoTime=t)
      parser.update_strings([dat])
      self.assertTrue(parser.can_valid)

  def test_packer_parser(self):

    signals = [
      ("COUNTER", "STEERING_CONTROL"),
      ("CHECKSUM", "STEERING_CONTROL"),
      ("STEER_TORQUE", "STEERING_CONTROL"),
      ("STEER_TORQUE_REQUEST", "STEERING_CONTROL"),

      ("Signal1", "Brake_Status"),

      ("COUNTER", "CAN_FD_MESSAGE"),
      ("64_BIT_LE", "CAN_FD_MESSAGE"),
      ("64_BIT_BE", "CAN_FD_MESSAGE"),
      ("SIGNED", "CAN_FD_MESSAGE"),
    ]

    packer = CANPacker(TEST_DBC)
    parser = CANParser(TEST_DBC, signals, [], 0, enforce_checks=False)

    for steer in range(-256, 255):
      for active in (1, 0):
        values = {
          "STEERING_CONTROL": {
            "STEER_TORQUE": steer,
            "STEER_TORQUE_REQUEST": active,
          },
          "Brake_Status": {
            "Signal1": 61042322657536.0,
          },
          "CAN_FD_MESSAGE": {
            "SIGNED": steer,
            "64_BIT_LE": random.randint(0, 100),
            "64_BIT_BE": random.randint(0, 100),
          },
        }

        msgs = [packer.make_can_msg(k, 0, v) for k, v in values.items()]
        bts = can_list_to_can_capnp(msgs)
        parser.update_strings([bts])

        for k, v in values.items():
          for key, val in v.items():
            self.assertAlmostEqual(parser.vl[k][key], val)

        # also check address
        for sig in ("STEER_TORQUE", "STEER_TORQUE_REQUEST", "COUNTER", "CHECKSUM"):
          self.assertEqual(parser.vl["STEERING_CONTROL"][sig], parser.vl[228][sig])

  def test_scale_offset(self):
    """Test that both scale and offset are correctly preserved"""
    dbc_file = "honda_civic_touring_2016_can_generated"

    signals = [
      ("USER_BRAKE", "VSA_STATUS"),
    ]
    checks = [("VSA_STATUS", 50)]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    for brake in range(0, 100):
      values = {"USER_BRAKE": brake}
      msgs = packer.make_can_msg("VSA_STATUS", 0, values)
      bts = can_list_to_can_capnp([msgs])

      parser.update_strings([bts])

      self.assertAlmostEqual(parser.vl["VSA_STATUS"]["USER_BRAKE"], brake)

  def test_subaru(self):
    # Subaru is little endian

    dbc_file = "subaru_global_2017_generated"

    signals = [
      ("COUNTER", "ES_LKAS"),
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
          "LKAS_Output": steer,
          "LKAS_Request": active,
          "SET_1": 1
        }

        msgs = packer.make_can_msg("ES_LKAS", 0, values)
        bts = can_list_to_can_capnp([msgs])
        parser.update_strings([bts])

        self.assertAlmostEqual(parser.vl["ES_LKAS"]["LKAS_Output"], steer)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["LKAS_Request"], active)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["SET_1"], 1)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["COUNTER"], idx % 16)
        idx += 1

  def test_bus_timeout(self):
    """Test CAN bus timeout detection"""
    dbc_file = "honda_civic_touring_2016_can_generated"

    freq = 100
    checks = [("VSA_STATUS", freq), ("STEER_MOTOR_TORQUE", freq/2)]

    parser = CANParser(dbc_file, [], checks, 0)
    packer = CANPacker(dbc_file)

    i = 0
    def send_msg(blank=False):
      nonlocal i
      i += 1
      t = i*((1 / freq) * 1e9)

      if blank:
        msgs = []
      else:
        msgs = [packer.make_can_msg("VSA_STATUS", 0, {}), ]

      can = can_list_to_can_capnp(msgs, logMonoTime=t)
      parser.update_strings([can, ])

    # all good, no timeout
    for _ in range(1000):
      send_msg()
      self.assertFalse(parser.bus_timeout, str(_))

    # timeout after 10 blank msgs
    for n in range(200):
      send_msg(blank=True)
      self.assertEqual(n >= 10, parser.bus_timeout)

    # no timeout immediately after seen again
    send_msg()
    self.assertFalse(parser.bus_timeout)

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
          can_msgs[frame].append(packer.make_can_msg("VSA_STATUS", 0, values))
          idx += 1

      can_strings = [can_list_to_can_capnp(msgs) for msgs in can_msgs]
      parser.update_strings(can_strings)
      vl_all = parser.vl_all["VSA_STATUS"]["USER_BRAKE"]

      self.assertEqual(vl_all, user_brake_vals)
      if len(user_brake_vals):
        self.assertEqual(vl_all[-1], parser.vl["VSA_STATUS"]["USER_BRAKE"])

  def test_timestamp_nanos(self):
    """Test message timestamp dict"""
    dbc_file = "honda_civic_touring_2016_can_generated"

    signals = [
      ("USER_BRAKE", "VSA_STATUS"),
      ("PEDAL_GAS", "POWERTRAIN_DATA"),
    ]
    checks = [
      ("VSA_STATUS", 50),
      ("POWERTRAIN_DATA", 100),
    ]

    parser = CANParser(dbc_file, signals, checks, 0)
    packer = CANPacker(dbc_file)

    # Check the default timestamp is zero
    for msg in ("VSA_STATUS", "POWERTRAIN_DATA"):
      ts_nanos = parser.ts_nanos[msg].values()
      self.assertEqual(set(ts_nanos), {0})

    # Check:
    # - timestamp is only updated for correct messages
    # - timestamp is correct for multiple runs
    # - timestamp is from the latest message if updating multiple strings
    for _ in range(10):
      can_strings = []
      log_mono_time = 0
      for i in range(10):
        log_mono_time = int(0.01 * i * 1e+9)
        can_msg = packer.make_can_msg("VSA_STATUS", 0, {})
        can_strings.append(can_list_to_can_capnp([can_msg], logMonoTime=log_mono_time))
      parser.update_strings(can_strings)

      ts_nanos = parser.ts_nanos["VSA_STATUS"].values()
      self.assertEqual(set(ts_nanos), {log_mono_time})
      ts_nanos = parser.ts_nanos["POWERTRAIN_DATA"].values()
      self.assertEqual(set(ts_nanos), {0})


if __name__ == "__main__":
  unittest.main()
