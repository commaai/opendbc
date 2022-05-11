#!/usr/bin/env python3
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
    dbc_times = []
    for dbc, lines in self.dbcs.items():
      start_time = time.monotonic()
      CANParser(dbc, [], [], 0)
      dbc_times.append((dbc, lines, time.monotonic() - start_time))

    # for dbc, lines, t in sorted(dbc_times, key=lambda x: x[-1]):
    #   print('{:54} ({:5} lines): {} ms'.format(dbc, lines, round(t * 1000, 4)))

    max_ms_per_line = max([t / lines * 1000 for _, lines, t in dbc_times])
    self.assertTrue(max_ms_per_line < 0.02, f'Max ms/line time too high: {max_ms_per_line} >= 0.02')


if __name__ == "__main__":
  unittest.main()
