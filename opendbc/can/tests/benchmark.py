#!/usr/bin/env python3
import time
from opendbc.can import CANPacker, CANParser


def _benchmark(checks, n):
  print()
  print("="*5, "Benchmark", "="*5)

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
        parser.update(m)
      t2 = time.process_time_ns()
    else:
      t1 = time.process_time_ns()
      for m in can_msgs:
        parser.update([m])
      t2 = time.process_time_ns()

    ets.append(t2 - t1)

  et = sum(ets) / len(ets)
  avg_nanos = et / len(can_msgs)
  print('%s: [%d] %.1fms to parse %s, avg: %dns' % ("_benchmark", n, et/1e6, len(can_msgs), avg_nanos))

if __name__ == "__main__":
  # python -m cProfile -s cumulative  benchmark.py
  _benchmark([('ACC_CONTROL', 10)], 1)
  _benchmark([('ACC_CONTROL', 10)], 10)
