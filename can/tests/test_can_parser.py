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
    for dbc, lines in self.dbcs.items():
      start_time = time.monotonic()
      CANParser(dbc, [], [], 0)
      total_time_ms = (time.monotonic() - start_time) * 1000

      self.assertTrue(total_time_ms < 45, f'{dbc}: total parsing time too high: {total_time_ms} ms >= 45 ms')
      ms_per_line = total_time_ms / lines
      self.assertTrue(ms_per_line < 0.02, f'{dbc}: max ms/line time too high: {ms_per_line} ms >= 0.02 ms')


if __name__ == "__main__":
  unittest.main()
