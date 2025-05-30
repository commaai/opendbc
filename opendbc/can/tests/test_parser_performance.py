import pytest
import time

from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker


@pytest.mark.skip("TODO: varies too much between machines")
class TestParser:
  def _benchmark(self, checks, thresholds, n):
    parser = CANParser('toyota_new_mc_pt_generated', checks, 0)
    packer = CANPacker('toyota_new_mc_pt_generated')

    t1 = time.process_time_ns()
    can_msgs = []
    for i in range(50000):
      values = {"ACC_CONTROL": {"ACC_TYPE": 1, "ALLOW_LONG_PRESS": 3}}
      msgs = [packer.make_can_msg(k, 0, v) for k, v in values.items()]
      can_msgs.append([int(0.01 * i * 1e9), msgs])
    t2 = time.process_time_ns()
    print(f'Pack time took {(t2 - t1) / 1e6} ms')

    ets = []
    for _ in range(25):
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
          parser.update_strings([m])
        t2 = time.process_time_ns()

      ets.append(t2 - t1)

    et = sum(ets) / len(ets)
    avg_nanos = et / len(can_msgs)
    print('%s: [%d] %.1fms to parse %s, avg: %dns' % (self._testMethodName, n, et/1e6, len(can_msgs), avg_nanos))

    minn, maxx = thresholds
    assert avg_nanos < maxx
    assert avg_nanos > minn, "Performance seems to have improved, update test thresholds."

  def test_performance_all_signals(self):
    self._benchmark([('ACC_CONTROL', 10)], (10000, 19000), 1)
    self._benchmark([('ACC_CONTROL', 10)], (1300, 5000), 10)
