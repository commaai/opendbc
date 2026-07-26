import tempfile
import unittest
from pathlib import Path

from opendbc.dbc.lint import lint_file, lint_paths


def lint_content(content: str):
  with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / "test.dbc"
    path.write_text(content)
    return lint_file(path)


class TestDBCLint(unittest.TestCase):
  def test_overlapping_little_endian_signals(self):
    errors = lint_content("""BO_ 1 TEST: 2 XXX\n SG_ FIRST : 0|8@1+ (1,0) [0|0] \"\" XXX\n SG_ SECOND : 4|8@1+ (1,0) [0|0] \"\" XXX\n""")
    self.assertEqual(len(errors), 1)
    self.assertIn("SECOND overlaps FIRST", errors[0].message)
    self.assertIn("bit(s) 4, 5, 6, 7", errors[0].message)

  def test_overlapping_motorola_signals(self):
    errors = lint_content("""BO_ 1 TEST: 2 XXX\n SG_ FIRST : 7|8@0+ (1,0) [0|0] \"\" XXX\n SG_ SECOND : 3|4@0+ (1,0) [0|0] \"\" XXX\n""")
    self.assertEqual(len(errors), 1)
    self.assertIn("SECOND overlaps FIRST", errors[0].message)

  def test_multiplexed_signals_may_overlap(self):
    errors = lint_content("""BO_ 1 TEST: 1 XXX
 SG_ SELECTOR M : 0|4@1+ (1,0) [0|0] "" XXX
 SG_ FIRST m0 : 4|4@1+ (1,0) [0|0] "" XXX
 SG_ SECOND m1 : 4|4@1+ (1,0) [0|0] "" XXX
""")
    self.assertEqual(errors, [])

  def test_other_layout_errors(self):
    errors = lint_content("""BO_ 1 TEST: 1 XXX
 SG_ DUPLICATE : 0|8@1+ (1,0) [0|0] "" XXX
 SG_ DUPLICATE : 7|2@1+ (1,0) [0|0] "" XXX
BO_ 1 AGAIN: 1 XXX
 SG_ OUT_OF_BOUNDS : 7|2@1+ (1,0) [0|0] "" XXX
""")
    messages = [error.message for error in errors]
    self.assertTrue(any("message address 1" in message for message in messages))
    self.assertTrue(any("signal DUPLICATE is already defined" in message for message in messages))
    self.assertTrue(any("OUT_OF_BOUNDS extends outside" in message for message in messages))

  def test_lints_dbc_files_in_directories(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "test.dbc"
      path.write_text("BO_ 1 TEST: 1 XXX\n SG_ ONE : 0|8@1+ (1,0) [0|0] \"\" XXX\n")
      self.assertEqual(lint_paths([directory]), [])
