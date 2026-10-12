import random
import unittest

from opendbc.safety.tests.mutation import SAFETY_DIR, _TestResult, enumerate_sites


class TestMutationKeys(unittest.TestCase):
  @staticmethod
  def sites(code, line=1):
    return enumerate_sites(f'# {line} "{SAFETY_DIR / "lateral.h"}"\n{code}')

  def test_line_numbers_and_whitespace(self):
    original = self.sites('int check(int a) {\n  return a > 0;\n}')
    moved = self.sites('\n\nint check(int a) {\n\n    return  a   >   0;\n}', line=20)
    self.assertEqual([s.cache_key for s in original], [s.cache_key for s in moved])
    self.assertNotEqual(original[0].origin_line, moved[0].origin_line)

  def test_unrelated_statement(self):
    original = self.sites('int check(int a) { return a > 0; }')
    changed = self.sites('int check(int a) { /* moved code */ a = 2; return a > 0; }')
    self.assertEqual([s.cache_key for s in original], [s.cache_key for s in changed])

  def test_repeated_expressions(self):
    sites = self.sites('int check(int a) { int x = a > 0; int y = a > 0; return x + y; }')
    comparisons = [s for s in sites if s.mutator == 'comparison']
    self.assertEqual(len(comparisons), 2)
    self.assertEqual(comparisons[0].snippet, comparisons[1].snippet)
    self.assertNotEqual(comparisons[0].cache_key, comparisons[1].cache_key)

  def test_expanded_macros(self):
    sites = self.sites('''int check(int a, int b) {
      int x = ({ int _a = a; int _b = b; (_a > _b) ? _a : _b; });
      int y = ({ int _a = b; int _b = a; (_a > _b) ? _a : _b; });
      return x + y;
    }''')
    comparisons = [s for s in sites if s.mutator == 'comparison']
    self.assertEqual(comparisons[0].snippet, comparisons[1].snippet)
    self.assertNotEqual(comparisons[0].cache_key, comparisons[1].cache_key)

  def test_repeated_expansion_in_statement(self):
    sites = self.sites('int check(int a) { return ((a > 0) + (a > 0)); }')
    comparisons = [s for s in sites if s.mutator == 'comparison']
    self.assertEqual(comparisons[0].snippet, comparisons[1].snippet)
    self.assertNotEqual(comparisons[0].cache_key, comparisons[1].cache_key)

  def test_repeatable_random_fixtures(self):
    result = _TestResult(5)

    def sample():
      result.startTest(self)
      try:
        return [random.getrandbits(32) for _ in range(5)]
      finally:
        result.stopTest(self)

    state = random.getstate()
    try:
      first = sample()
      random.seed(42)
      self.assertEqual(first, sample())
    finally:
      random.setstate(state)


if __name__ == '__main__':
  unittest.main()
