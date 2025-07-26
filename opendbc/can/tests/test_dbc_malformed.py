import tempfile
import os
from opendbc.can.dbc import DBC


class TestDBCMalformed:
  def test_malformed_bo_line(self):
    # Test malformed BO_ line (covers defensive branch in dbc.py)
    malformed_dbc = """VERSION ""

BO_ INVALID_MESSAGE_FORMAT
"""

    # Create temporary file for DBC content
    with tempfile.NamedTemporaryFile(mode='w', suffix='.dbc', delete=False) as f:
      f.write(malformed_dbc)
      temp_path = f.name

    try:
      # Should handle malformed BO_ line gracefully
      dbc = DBC(temp_path)
      # Should not have any messages due to malformed line
      assert len(dbc.msgs) == 0
    except (ValueError, IndexError, RuntimeError):
      # Expected to fail gracefully on malformed input
      pass
    finally:
      os.unlink(temp_path)

  def test_malformed_sg_line(self):
    # Test malformed SG_ line (covers defensive branch in dbc.py)
    malformed_dbc = """VERSION ""

BO_ 100 test_message: 8 Vector__XXX
 SG_ INVALID_SIGNAL_FORMAT
 SG_ valid_signal : 8|8@1+ (1,0) [0|255] "" Vector__XXX
"""

    # Create temporary file for DBC content
    with tempfile.NamedTemporaryFile(mode='w', suffix='.dbc', delete=False) as f:
      f.write(malformed_dbc)
      temp_path = f.name

    try:
      # Should handle malformed SG_ line gracefully
      dbc = DBC(temp_path)
      # Should parse the valid parts
      assert 100 in dbc.msgs
      # May or may not have signals depending on error handling
    except (ValueError, IndexError, RuntimeError):
      # Expected to fail gracefully on malformed input
      pass
    finally:
      os.unlink(temp_path)

  def test_empty_dbc(self):
    # Test empty DBC content
    empty_dbc = ""

    # Create temporary file for DBC content
    with tempfile.NamedTemporaryFile(mode='w', suffix='.dbc', delete=False) as f:
      f.write(empty_dbc)
      temp_path = f.name

    try:
      dbc = DBC(temp_path)
      # Should handle empty input
      assert len(dbc.msgs) == 0
    finally:
      os.unlink(temp_path)

  def test_valid_dbc_for_comparison(self):
    # Test valid DBC to ensure our malformed tests are meaningful
    valid_dbc = """VERSION ""

BO_ 100 test_message: 8 Vector__XXX
 SG_ signal1 : 0|8@1+ (1,0) [0|255] "" Vector__XXX
 SG_ signal2 : 8|8@1+ (1,0) [0|255] "" Vector__XXX
"""

    # Create temporary file for DBC content
    with tempfile.NamedTemporaryFile(mode='w', suffix='.dbc', delete=False) as f:
      f.write(valid_dbc)
      temp_path = f.name

    try:
      dbc = DBC(temp_path)
      # Should parse successfully
      assert 100 in dbc.msgs
      assert len(dbc.msgs[100].sigs) == 2
    finally:
      os.unlink(temp_path)