#!/usr/bin/env python3
import glob
import os
import unittest

from opendbc import DBC_PATH
from opendbc.can.parser import CANParser


class TestDBCParser(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.dbcs = []
    for dbc in glob.glob(f"{DBC_PATH}/*.dbc"):
      cls.dbcs.append(os.path.basename(dbc).split('.')[0])

  def test_parse_all_dbcs(self):
    for dbc in self.dbcs:
      print(dbc)
      CANParser(dbc, [], [], 0)


if __name__ == "__main__":
  unittest.main()
