#!/usr/bin/env python3
import time
import unittest

from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from opendbc.can.tests.test_packer_parser import can_list_to_can_capnp



class TestParser(unittest.TestCase):
  def _benchmark(self, signals, checks, msg):
    parser = CANParser('toyota_new_mc_pt_generated', signals, checks, 0, False)
    packer = CANPacker('toyota_new_mc_pt_generated')
    can_msgs = []
    for i in range(10000):
      values = {"ACC_CONTROL": {"ACC_TYPE": 1, "ALLOW_LONG_PRESS": 3}}
      msgs = [packer.make_can_msg(k, 0, v) for k, v in values.items()]
      bts = can_list_to_can_capnp(msgs, logMonoTime=int(0.01 * i * 1e9))
      can_msgs.append(bts)

    t1 = time.process_time()
    for m in can_msgs:
      parser.update_string(m)
    ts = (time.process_time() - t1) * 1000


    print("-"*25)
    print(self._testMethodName)
    print('%s: %.3f ms total: %s avg: %f' % (msg, ts, len(can_msgs), ts / len(can_msgs)))
    print("-"*25)

  def test_performance_one_signal(self):
    self._benchmark([("ACC_TYPE", "ACC_CONTROL")], [('ACC_CONTROL', 10)], 'One Signal')

  def test_performance_all_signals(self):
    signals = [
        ("ACCEL_CMD", "ACC_CONTROL"),
        ("ALLOW_LONG_PRESS", "ACC_CONTROL"),
        ("ACC_MALFUNCTION", "ACC_CONTROL"),
        ("RADAR_DIRTY", "ACC_CONTROL"),
        ("DISTANCE", "ACC_CONTROL"),
        ("MINI_CAR", "ACC_CONTROL"),
        ("CANCEL_REQ", "ACC_CONTROL"),
        ("ACC_CUT_IN", "ACC_CONTROL"),
        ("PERMIT_BRAKING", "ACC_CONTROL"),
        ("RELEASE_STANDSTILL", "ACC_CONTROL"),
        ("ITS_CONNECT_LEAD", "ACC_CONTROL"),
        ("ACCEL_CMD_ALT", "ACC_CONTROL"),
        ("CHECKSUM", "ACC_CONTROL"),
    ]
    self._benchmark(signals, [('ACC_CONTROL', 10)], 'All Signals')


if __name__ == "__main__":
  unittest.main()
