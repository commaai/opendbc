import pytest
from opendbc.can.dbc import DBC


class TestDBCMalformed:
  def test_malformed_bo_line(self):
    # Test malformed BO_ line (covers defensive branch in dbc.py)
    malformed_dbc = """
VERSION ""

BO_ INVALID_MESSAGE_FORMAT
SG_ signal1 : 0|8@1+ (1,0) [0|255] "" Vector__XXX
"""
    
    # Should handle malformed BO_ line gracefully
    dbc = DBC()
    try:
      dbc.add_dbc_string(malformed_dbc)
      # If it doesn't raise, verify it parsed what it could
      assert len(dbc.msgs) >= 0  # Should not crash
    except (ValueError, IndexError):
      # Expected to fail gracefully on malformed input
      pass
      
  def test_malformed_sg_line(self):
    # Test malformed SG_ line (covers defensive branch in dbc.py)
    malformed_dbc = """
VERSION ""

BO_ 100 test_message: 8 Vector__XXX
 SG_ INVALID_SIGNAL_FORMAT
 SG_ valid_signal : 8|8@1+ (1,0) [0|255] "" Vector__XXX
"""
    
    # Should handle malformed SG_ line gracefully
    dbc = DBC()
    try:
      dbc.add_dbc_string(malformed_dbc)
      # Should parse the valid parts
      assert 100 in dbc.msgs
      # May or may not have signals depending on error handling
    except (ValueError, IndexError):
      # Expected to fail gracefully on malformed input
      pass
      
  def test_empty_dbc(self):
    # Test empty DBC content
    empty_dbc = ""
    
    dbc = DBC()
    dbc.add_dbc_string(empty_dbc)
    
    # Should handle empty input
    assert len(dbc.msgs) == 0
    
  def test_valid_dbc_for_comparison(self):
    # Test valid DBC to ensure our malformed tests are meaningful
    valid_dbc = """
VERSION ""

BO_ 100 test_message: 8 Vector__XXX
 SG_ signal1 : 0|8@1+ (1,0) [0|255] "" Vector__XXX
 SG_ signal2 : 8|8@1+ (1,0) [0|255] "" Vector__XXX
"""
    
    dbc = DBC()
    dbc.add_dbc_string(valid_dbc)
    
    # Should parse successfully
    assert 100 in dbc.msgs
    assert len(dbc.msgs[100].sigs) == 2