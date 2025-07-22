import pytest

from opendbc.can.parser import CANParser, CANDefine
from opendbc.can.packer import CANPacker
from opendbc.can.tests import TEST_DBC


class TestCanParserPackerExceptions:
  def test_civic_exceptions(self):
    dbc_file = "honda_civic_touring_2016_can_generated"
    dbc_invalid = dbc_file + "abcdef"
    with pytest.raises(RuntimeError):
      CANPacker(dbc_invalid)
    with pytest.raises(RuntimeError):
      CANDefine(dbc_invalid)
    with pytest.raises(KeyError):
      CANDefine(TEST_DBC)

    parser = CANParser(dbc_file, 0)
    with pytest.raises(RuntimeError):
      parser.update_strings([b''])

    # Everything is supposed to work below
    CANParser(dbc_file, 0)
    CANPacker(dbc_file)
    CANDefine(dbc_file)
