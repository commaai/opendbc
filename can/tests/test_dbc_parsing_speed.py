#!/usr/bin/env python3
import os
import unittest

from opendbc.can.parser import CANParser
import time
from opendbc.generator.generator import generated_suffix
import glob
from opendbc import DBC_PATH


class TestParsingSpeed(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.dbcs = []
    for dbc in glob.glob(f"{DBC_PATH}/*{generated_suffix}"):
      cls.dbcs.append(os.path.basename(dbc).split('.')[0])

  def test_parsing_speed(self):
    print('test_parsing_speed')
    print(self.dbcs)
    start_time = time.time()
    for dbc in self.dbcs:
      CANParser(dbc, [], [], 0)
    elapsed = time.time() - start_time
    self.assertLess(elapsed, 0.15, "Took too long to parse {} DBCs".format(len(self.dbcs)))


if __name__ == "__main__":
  unittest.main()
