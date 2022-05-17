#!/usr/bin/env python3
from collections import defaultdict
import glob
import os
import time
import unittest

from opendbc import DBC_PATH
from opendbc.can.parser import CANParser


class TestCANParser(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.dbcs = {}
    for dbc in glob.glob(f"{DBC_PATH}/*.dbc"):
      dbc_name = os.path.basename(dbc).split('.')[0]
      with open(dbc, 'r') as f:
        cls.dbcs[dbc_name] = len(f.readlines())

  def test_dbc_parsing_speed(self):
    dbc_parsing_times = defaultdict(lambda: defaultdict(list))
    for _ in range(100):
      for dbc, lines in self.dbcs.items():
        start_time = time.monotonic()
        CANParser(dbc, [], [], 0)
        total_time_ms = (time.monotonic() - start_time) * 1000
        dbc_parsing_times[dbc]['total'].append(total_time_ms)
        dbc_parsing_times[dbc]['lines'].append(total_time_ms / lines)

    for dbc, dbc_times in dbc_parsing_times.items():
      # FIXME: this doesn't really test anything since it caches the parsed dbc after the first run
      self.assertTrue(min(dbc_times['total']) < 45, f'{dbc}: total parsing time too high: {min(dbc_times["total"])} ms >= 45 ms')
      self.assertTrue(min(dbc_times['lines']) < 0.02, f'{dbc}: max ms/line time too high: {min(dbc_times["lines"])} ms >= 0.02 ms')


if __name__ == "__main__":
  unittest.main()
