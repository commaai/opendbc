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
      dbc_times.append((time.monotonic() - start_time) / lines * 1000)

    self.assertTrue(max(dbc_times) < 0.15, f'Max ms/line time too high: {max(dbc_times)} > 0.1')


if __name__ == "__main__":
  unittest.main()
