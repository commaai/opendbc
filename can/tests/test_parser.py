#!/usr/bin/env python3
import time
import unittest

from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from opendbc.can.tests.test_packer_parser import can_list_to_can_capnp



class TestParser(unittest.TestCase):
  def _benchmark(self, signals, checks, thresholds, n):
    parser = CANParser('toyota_new_mc_pt_generated', signals, checks, 0, False)
    packer = CANPacker('toyota_new_mc_pt_generated')
    can_msgs = []
    for i in range(50000):
      values = {"ACC_CONTROL": {"ACC_TYPE": 1, "ALLOW_LONG_PRESS": 3}}
      msgs = [packer.make_can_msg(k, 0, v) for k, v in values.items()]
      bts = can_list_to_can_capnp(msgs, logMonoTime=int(0.01 * i * 1e9))
      can_msgs.append(bts)

    if n > 1:
      strings = []
      for i in range(0, len(can_msgs), n):
        strings.append(can_msgs[i:i + n])
      t1 = time.process_time_ns()
      for m in strings:
        parser.update_strings(m)
      t2 = time.process_time_ns()
    else:
      t1 = time.process_time_ns()
      for m in can_msgs:
        parser.update_string(m)
      t2 = time.process_time_ns()

    et = t2 - t1
    avg_nanos = et / len(can_msgs)
    method = 'update_strings'  if n > 1 else 'update_string'
    print('%s: [%s] %.1fms to parse %s, avg: %dns' % (self._testMethodName, method, et/1e6, len(can_msgs), avg_nanos))

    minn, maxx = thresholds
    self.assertLess(avg_nanos, maxx)
    self.assertGreater(avg_nanos, minn, "Performance seems to have improved, update test thresholds.")

  def test_performance_one_signal(self):
    signals = [
      ("ACCEL_CMD", "ACC_CONTROL"),
    ]
    self._benchmark(signals, [('ACC_CONTROL', 10)], (5000, 7000), 1)
    self._benchmark(signals, [('ACC_CONTROL', 10)], (2200, 3300), 10)

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
    self._benchmark(signals, [('ACC_CONTROL', 10)], (12000, 19000), 1)
    self._benchmark(signals, [('ACC_CONTROL', 10)], (7000, 13000), 10)


if __name__ == "__main__":
  unittest.main()
