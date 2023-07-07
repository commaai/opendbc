#!/usr/bin/env python3
import unittest

from opendbc.can.parser import CANParser
from opendbc.can.tests import ALL_DBCS


class TestDBCParser(unittest.TestCase):
  def test_parse_all_dbcs(self):
    """
      Dynamic DBC parser checks:
        - Checksum and counter length, start bit, endianness
        - Duplicate message addresses and names
        - Signal out of bounds
        - All BO_, SG_, VAL_ lines for syntax errors
    """

    for dbc in ALL_DBCS:
      with self.subTest(dbc=dbc):
        CANParser(dbc, [], [], 0)


if __name__ == "__main__":
  unittest.main()
