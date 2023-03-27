#!/usr/bin/env python3
import time
import unittest

from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from opendbc.can.tests.test_packer_parser import can_list_to_can_capnp



class TestParser(unittest.TestCase):
  def _benchmark(self, signals, checks, thresholds):
    parser = CANParser('toyota_new_mc_pt_generated', signals, checks, 0, False)
    packer = CANPacker('toyota_new_mc_pt_generated')
    can_msgs = []
    for i in range(10000):
      values = {"ACC_CONTROL": {"ACC_TYPE": 1, "ALLOW_LONG_PRESS": 3}}
      msgs = [packer.make_can_msg(k, 0, v) for k, v in values.items()]
      bts = can_list_to_can_capnp(msgs, logMonoTime=int(0.01 * i * 1e9))
      can_msgs.append(bts)

    t1 = time.process_time_ns()
    for m in can_msgs:
      parser.update_string(m)
    t2 = time.process_time_ns()
    et = t2 - t1
    avg_nanos = et / len(can_msgs)
    print('%s: %.1fms to parse %s, avg: %dns' % (self._testMethodName, et/1e6, len(can_msgs), avg_nanos))

    minn, maxx = thresholds
    self.assertLess(avg_nanos, maxx)
    self.assertGreater(avg_nanos, minn, "Performance seems to have improved, update test thresholds.")

  def test_performance_one_signal(self):
    signals = [
      ("ACCEL_CMD", "ACC_CONTROL"),
    ]
    self._benchmark(signals, [('ACC_CONTROL', 10)], (8500, 10000))

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
    self._benchmark(signals, [('ACC_CONTROL', 10)], (18000, 20000))


if __name__ == "__main__":
  unittest.main()
