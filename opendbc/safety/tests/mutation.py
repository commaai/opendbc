#!/usr/bin/env python3
"""Mutation testing for the safety code.

Every operator and comparison literal in opendbc/safety is mutated, one at a time, and the
safety tests are run against each mutant. All mutants are compiled into one library and
selected at runtime, so the build cost is paid once.

A mutant is only "killed" when a test assertion fails. Crashes, hangs and test exceptions
are reported as their own outcomes so that a broken harness can never pass as a kill, and the
whole suite is first run against the instrumented library with no mutant active.
"""
import argparse
import ctypes
import json
import os
import pickle
import re
import resource
import select
import signal
import subprocess
import sys
import tempfile
import time
import traceback
import unittest
from collections import Counter, defaultdict, deque
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, as_completed, wait
from dataclasses import dataclass, replace
from itertools import zip_longest
from pathlib import Path

import tree_sitter_c as ts_c
import tree_sitter as ts

from opendbc.safety.tests.common import SafetyTestBase
from opendbc.safety.tests.libsafety import libsafety_py


ROOT = Path(__file__).resolve().parents[3]
SAFETY_DIR = ROOT / "opendbc" / "safety"
SAFETY_MODES_DIR = SAFETY_DIR / "modes"
SAFETY_TESTS_DIR = SAFETY_DIR / "tests"
SAFETY_C = SAFETY_TESTS_DIR / "libsafety" / "safety.c"

# Outcomes. Only KILLED counts as the tests detecting the mutant.
KILLED = "killed"      # a test assertion failed
SURVIVED = "survived"  # every targeted test passed
CRASHED = "crashed"    # the test process died from a signal (segfault, SIGFPE, ...)
TIMEOUT = "timeout"    # a single test exceeded its time budget
ERROR = "error"        # a test raised an exception, or the harness itself failed
OUTCOMES = (KILLED, SURVIVED, CRASHED, TIMEOUT, ERROR)

# A test may take this many times the slowest baseline test before the mutant is declared hung.
TEST_TIMEOUT_FACTOR = 10
TEST_TIMEOUT_MIN_SEC = 5.0
# Backstop for the whole test process. Only reachable if something hangs outside a test.
PROCESS_TIMEOUT_SEC = 600.0
# Tests that killed earlier mutants in the same file are run first; this many of them
KILLER_HINTS = 10
# The test that killed each mutant last run, keyed by file, mutator and mutated expression. Run first next time.
KILLER_CACHE = SAFETY_TESTS_DIR / ".mutation_killers.json"
# Mutants that cannot change observable behaviour, keyed the same way, with the reason next to them
EQUIVALENT_MUTANTS = SAFETY_TESTS_DIR / "mutation_equivalents.txt"

ANSI_RESET = "\033[0m"
ANSI_BOLD = "\033[1m"
ANSI_RED = "\033[31m"
ANSI_GREEN = "\033[32m"
ANSI_YELLOW = "\033[33m"

COMPARISON_OPERATORS = {
  "==": "!=",
  "!=": "==",
  ">": "<=",
  ">=": "<",
  "<": ">=",
  "<=": ">",
}

# mutator -> (tree-sitter node type, {original operator: mutated operator})
OPERATOR_MUTATORS = {
  "increment": ("update_expression", {"++": "--"}),
  "decrement": ("update_expression", {"--": "++"}),
  "comparison": ("binary_expression", COMPARISON_OPERATORS),
  "bitwise_assignment": ("assignment_expression", {"&=": "|=", "|=": "&=", "^=": "&="}),
  "bitwise": ("binary_expression", {"&": "|", "|": "&", "^": "&"}),
  "arithmetic_assignment": ("assignment_expression", {"+=": "-=", "-=": "+=", "*=": "/=", "/=": "*=", "%=": "*="}),
  "arithmetic": ("binary_expression", {"+": "-", "-": "+", "*": "/", "/": "*", "%": "*"}),
  "remove_negation": ("unary_expression", {"!": ""}),
}
# "boundary" adds one to integer literals that are operands of a comparison
MUTATORS = (*OPERATOR_MUTATORS, "boundary")


@dataclass(frozen=True)
class MutationSite:
  site_id: int
  mutator: str
  original_op: str
  mutated_op: str
  # byte offsets into the preprocessed source
  expr_start: int
  expr_end: int
  op_start: int
  op_end: int
  # where the site came from
  origin_file: Path
  origin_line: int
  snippet: str  # the mutated expression, from the preprocessed source
  # mutating a constant expression initializer doesn't compile
  constexpr: bool

  @property
  def location(self):
    return f"{self.origin_file.relative_to(ROOT)}:{self.origin_line}"

  @property
  def is_mode(self):
    return SAFETY_MODES_DIR in self.origin_file.parents

  @property
  def cache_key(self):
    return f"{self.origin_file.relative_to(ROOT)} {self.mutator} {self.snippet}"


@dataclass(frozen=True)
class TestOutcome:
  outcome: str
  detail: str
  test_id: str  # the test that failed or errored, if any
  tests_run: int
  slowest_test_sec: float


@dataclass(frozen=True)
class MutantResult:
  site: MutationSite
  outcome: TestOutcome
  test_sec: float


def colorize(text, color):
  term = os.environ.get("TERM", "")
  if not sys.stdout.isatty() or term in ("", "dumb") or "NO_COLOR" in os.environ:
    return text
  return f"{color}{text}{ANSI_RESET}"


def format_mutation(site):
  return colorize(f"{site.original_op}->{site.mutated_op}", ANSI_RED)


# ***** site discovery *****

def _parse_int_literal(token):
  m = re.fullmatch(r"([0-9][0-9a-fA-FxX]*)([uUlL]*)", token)
  if m is None:
    return None
  body, suffix = m.groups()
  try:
    value = int(body, 0)
  except ValueError:
    return None
  if body.lower().startswith("0x"):
    return f"0x{value + 1:X}{suffix}"
  return f"{value + 1}{suffix}"


def _is_in_constexpr_context(node):
  """Check if a node is inside a static or file-scope variable initializer."""
  current = node.parent
  while current is not None:
    if current.type == "init_declarator":
      decl = current.parent
      if decl and decl.type == "declaration":
        for child in decl.children:
          if child.type == "storage_class_specifier" and child.text == b"static":
            return True
        if decl.parent and decl.parent.type == "translation_unit":
          return True
    current = current.parent
  return False


def _unparenthesize(node):
  while node.type == "parenthesized_expression":
    node = node.named_children[0]
  return node


def _is_constant_arithmetic(node):
  """Both operands are integer literals, e.g. the (b) / 8U inside an expanded GET_BIT(msg, b).

  Mutating these mutates the macro rather than the safety logic, and the result is mostly
  equivalent or undefined (out-of-range shifts and indices), so it can't be tested reliably.
  """
  operands = [node.child_by_field_name(field) for field in ("left", "right")]
  return all(op is not None and _unparenthesize(op).type == "number_literal" for op in operands)


def _is_null_pointer(literal):
  """The 0 in NULL, ((void *)0). No other pointer value compares equal, so (void *)1 is equivalent."""
  cast = literal.parent
  return cast is not None and cast.type == "cast_expression" and b"*" in cast.child_by_field_name("type").text


def _prepare_for_parsing(txt):
  """Blank line markers and replace __typeof__() for tree-sitter. Preserves byte offsets."""
  result = re.sub(
    r'^[ \t]*#[ \t]+\d+[ \t]+"[^\n]*',
    lambda m: " " * len(m.group()),
    txt,
    flags=re.MULTILINE,
  )
  # Replace __typeof__(...) with padded int (handle nested parens)
  parts = []
  i = 0
  for m in re.finditer(r"(?:__typeof__|typeof)\s*\(", result):
    if m.start() < i:
      continue  # skip nested typeof inside already-replaced region
    parts.append(result[i:m.start()])
    depth = 1
    j = m.end()
    while j < len(result) and depth > 0:
      if result[j] == "(":
        depth += 1
      elif result[j] == ")":
        depth -= 1
      j += 1
    parts.append("int" + " " * (j - m.start() - 3))
    i = j
  parts.append(result[i:])
  return "".join(parts)


def preprocess(output_file):
  subprocess.run([
    "cc", "-E", "-std=gnu11", "-nostdlib", "-fno-builtin", "-DALLOW_DEBUG",
    f"-I{ROOT}", str(SAFETY_C), "-o", str(output_file),
  ], cwd=ROOT, capture_output=True, check=True)
  txt = output_file.read_text()
  # tree-sitter reports byte offsets, which we use to index the text
  assert txt.isascii(), "preprocessed source must be ASCII"
  return txt


def _line_map(txt):
  """Map each line of the preprocessed source back to (file, line) using the `# N "file"` markers."""
  line_map = {}
  current_file, current_line = None, None
  directive_re = re.compile(r'^\s*#\s*(\d+)\s+"([^"]+)"')
  for pp_line_num, pp_line in enumerate(txt.split("\n"), start=1):
    m = directive_re.match(pp_line)
    if m:
      current_line = int(m.group(1))
      current_file = Path(m.group(2)).resolve()
    elif current_file is not None:
      line_map[pp_line_num] = (current_file, current_line)
      current_line += 1
  return line_map


def _snippet(txt, context, op_start, op_end, mutated_op, max_len=80):
  """The mutated expression as it will appear in the preprocessed source, with the mutation marked."""
  before = " ".join(txt[context.start_byte:op_start].split())
  after = " ".join(txt[op_end:context.end_byte].split())
  if len(before) > max_len:
    before = "..." + before[-max_len:]
  if len(after) > max_len:
    after = after[:max_len] + "..."
  return " ".join(part for part in (before, f"[[{txt[op_start:op_end]}->{mutated_op}]]", after) if part)


def enumerate_sites(txt):
  line_map = _line_map(txt)
  parser = ts.Parser(ts.Language(ts_c.language()))
  parse_txt = _prepare_for_parsing(txt)
  tree = parser.parse(parse_txt.encode())

  sites = []

  def add_site(mutator, original_op, mutated_op, expr, op_start, op_end, context):
    pp_line_num = expr.start_point[0] + 1
    mapped = line_map.get(pp_line_num)
    if mapped is None:
      return
    origin_file, origin_line = mapped
    # only mutate the safety code itself, not the test helpers in safety.c
    if SAFETY_DIR not in origin_file.parents or SAFETY_TESTS_DIR in origin_file.parents:
      return
    sites.append(MutationSite(
      site_id=-1, mutator=mutator, original_op=original_op, mutated_op=mutated_op,
      expr_start=expr.start_byte, expr_end=expr.end_byte, op_start=op_start, op_end=op_end,
      origin_file=origin_file, origin_line=origin_line,
      snippet=_snippet(parse_txt, context, op_start, op_end, mutated_op),
      constexpr=_is_in_constexpr_context(expr),
    ))

  seen_literals = set()
  stack = [tree.root_node]
  while stack:
    node = stack.pop()
    stack.extend(node.children)
    op = node.child_by_field_name("operator")
    if op is None:
      continue

    for mutator, (node_type, op_map) in OPERATOR_MUTATORS.items():
      if node.type == node_type and op.type in op_map and not (node.type == "binary_expression" and _is_constant_arithmetic(node)):
        add_site(mutator, op.type, op_map[op.type], node, op.start_byte, op.end_byte, context=node)

    # boundary: integer literals anywhere inside the operands of a comparison
    if node.type == "binary_expression" and op.type in COMPARISON_OPERATORS:
      operands = [node.child_by_field_name(field) for field in ("left", "right")]
      while operands:
        n = operands.pop()
        operands.extend(n.children)
        if n.type != "number_literal" or (n.start_byte, n.end_byte) in seen_literals or _is_null_pointer(n):
          continue
        seen_literals.add((n.start_byte, n.end_byte))
        token = txt[n.start_byte:n.end_byte]
        mutated = _parse_int_literal(token)
        if mutated is not None:
          add_site("boundary", token, mutated, n, n.start_byte, n.end_byte, context=node)

  sites.sort(key=lambda s: (s.op_start, s.mutator))
  sites = [replace(s, site_id=i) for i, s in enumerate(sites)]
  assert len({(s.op_start, s.op_end, s.mutator) for s in sites}) == len(sites), "duplicate mutation sites"
  return sites


# ***** instrumentation *****

def _instrument_source(source, sites):
  """Wrap every site as ((__mutation_active_id == id) ? mutated : original), nesting as needed."""
  # Sort by start ascending, end descending (outermost first when same start)
  sorted_sites = sorted(sites, key=lambda s: (s.expr_start, -s.expr_end))

  # Build containment forest using a stack
  roots = []
  stack = []
  for site in sorted_sites:
    while stack and stack[-1][0].expr_end <= site.expr_start:
      stack.pop()
    node = [site, []]
    if stack:
      stack[-1][1].append(node)
    else:
      roots.append(node)
    stack.append(node)

  def build_replacement(site, children):
    parts = []
    pos = site.expr_start
    op_rel = None
    running_len = 0

    for child_site, child_children in children:
      seg = source[pos : child_site.expr_start]
      if op_rel is None and site.op_start >= pos and site.op_start < child_site.expr_start:
        op_rel = running_len + (site.op_start - pos)
      parts.append(seg)
      running_len += len(seg)

      child_repl = build_replacement(child_site, child_children)
      parts.append(child_repl)
      running_len += len(child_repl)
      pos = child_site.expr_end

    seg = source[pos : site.expr_end]
    if op_rel is None and site.op_start >= pos:
      op_rel = running_len + (site.op_start - pos)
    parts.append(seg)

    expr_text = "".join(parts)
    op_len = site.op_end - site.op_start
    assert op_rel is not None and expr_text[op_rel : op_rel + op_len] == site.original_op, (
      f"Operator mismatch (site_id={site.site_id}): expected {site.original_op!r} at offset {op_rel}"
    )
    mutated_expr = f"{expr_text[:op_rel]}{site.mutated_op}{expr_text[op_rel + op_len :]}"
    return f"((__mutation_active_id == {site.site_id}) ? ({mutated_expr}) : ({expr_text}))"

  result_parts = []
  pos = 0
  for site, children in roots:
    result_parts.append(source[pos : site.expr_start])
    result_parts.append(build_replacement(site, children))
    pos = site.expr_end
  result_parts.append(source[pos:])
  return "".join(result_parts)


def compile_mutated_library(preprocessed_source, sites, output_so):
  prelude = """
    static int __mutation_active_id = -1;
    void mutation_set_active_mutant(int id) { __mutation_active_id = id; }
  """
  mutation_source = output_so.with_suffix(".c")
  mutation_source.write_text(prelude + _instrument_source(preprocessed_source, sites))

  subprocess.run([
    "cc", "-shared", "-fPIC", "-w", "-fno-builtin", "-std=gnu11",
    "-g0", "-O0", "-DALLOW_DEBUG",
    str(mutation_source), "-o", str(output_so),
  ], cwd=ROOT, check=True)


# ***** test selection *****

def load_tests():
  """All libsafety tests, keyed by test module file name."""
  loader = unittest.TestLoader()
  tests = {}
  seen = set()
  for test_file in sorted(SAFETY_TESTS_DIR.glob("test_*.py")):
    module_name = ".".join(test_file.relative_to(ROOT).with_suffix("").parts)
    suite = loader.loadTestsFromName(module_name)
    # base classes imported from another test module are picked up twice; keep the first
    tests[test_file.name] = [t for t in _flatten(suite) if isinstance(t, SafetyTestBase) and t.id() not in seen]
    seen.update(t.id() for t in tests[test_file.name])
  assert not loader.errors, "\n".join(loader.errors)
  return tests


def _flatten(suite):
  for test in suite:
    if isinstance(test, unittest.TestSuite):
      yield from _flatten(test)
    else:
      yield test


def _order_core_tests(tests):
  """Test order for sites in shared code: round-robin across methods, most widely shared method first.

  Methods inherited by many classes exercise the most fundamental safety logic,
  so with failfast this finds a kill quickly.
  """
  by_method = defaultdict(list)
  for module in sorted(tests):
    for t in tests[module]:
      by_method[t.id().rsplit(".", 1)[-1]].append(t)
  methods = sorted(by_method, key=lambda m: -len(by_method[m]))
  return [t for rnd in zip_longest(*(by_method[m] for m in methods)) for t in rnd if t is not None]


def tests_for_site(site, tests, core_tests, first_tests=()):
  """Ordered tests for a site. first_tests are ids of tests to run before all others, best first."""
  if not site.is_mode:
    ordered = core_tests
  else:
    # test_<mode>.py, or for a shared header (hyundai_common.h) every module of that brand
    stem = site.origin_file.stem
    modules = [f"test_{stem}.py"] if f"test_{stem}.py" in tests else sorted(m for m in tests if m.startswith(f"test_{stem.split('_')[0]}"))
    assert modules, f"no test module for {site.origin_file}"
    ordered = [t for m in modules for t in tests[m]]
  rank = {test_id: i for i, test_id in enumerate(first_tests)}
  return sorted(ordered, key=lambda t: rank.get(t.id(), len(rank)))


# ***** test execution *****

class _TestResult(unittest.TestResult):
  """Fail-fast result that arms a watchdog around every test, so a hung mutant dies by SIGALRM."""

  def __init__(self, test_timeout_sec):
    super().__init__()
    self.failfast = True
    self.test_timeout_sec = test_timeout_sec
    self.slowest_test_sec = 0.0

  def startTest(self, test):
    super().startTest(test)
    self._t0 = time.perf_counter()
    signal.setitimer(signal.ITIMER_REAL, self.test_timeout_sec)

  def stopTest(self, test):
    signal.setitimer(signal.ITIMER_REAL, 0)
    self.slowest_test_sec = max(self.slowest_test_sec, time.perf_counter() - self._t0)
    super().stopTest(test)


def run_tests(tests, lib_path, mutant_id, test_timeout_sec):
  libsafety_py.load(lib_path)
  libsafety_py.libsafety.mutation_set_active_mutant(mutant_id)

  result = _TestResult(test_timeout_sec)
  unittest.TestSuite(tests).run(result)

  if result.failures:
    test, tb = result.failures[0]
    return TestOutcome(KILLED, tb.strip().splitlines()[-1], test.id(), result.testsRun, result.slowest_test_sec)
  if result.errors:
    test, tb = result.errors[0]
    return TestOutcome(ERROR, tb.strip().splitlines()[-1], test.id(), result.testsRun, result.slowest_test_sec)
  return TestOutcome(SURVIVED, "", "", result.testsRun, result.slowest_test_sec)


def run_isolated(fn, timeout_sec):
  """Run fn() in a forked child so a crash or hang can't take down the worker.

  Returns fn's TestOutcome, or a CRASHED / TIMEOUT / ERROR outcome describing what happened to the child.
  """
  rfd, wfd = os.pipe()
  pid = os.fork()
  if pid == 0:
    os.close(rfd)
    try:
      # a crash is an expected outcome: don't waste seconds dumping core or waiting on the system crash handler
      resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
      if sys.platform == "linux":
        ctypes.CDLL(None).prctl(4, 0)  # PR_SET_DUMPABLE
      try:
        payload = fn()
      except BaseException:
        payload = TestOutcome(ERROR, traceback.format_exc(), "", 0, 0.0)
      with os.fdopen(wfd, "wb") as f:
        pickle.dump(payload, f)
    finally:
      os._exit(0)

  os.close(wfd)
  with os.fdopen(rfd, "rb") as f:
    data = f.read() if select.select([f], [], [], timeout_sec)[0] else None
  if data is None:
    os.kill(pid, signal.SIGKILL)
  _, status = os.waitpid(pid, 0)

  if data is None:
    return TestOutcome(TIMEOUT, f"no result after {timeout_sec:.0f}s", "", 0, 0.0)
  if os.WIFSIGNALED(status):
    sig = os.WTERMSIG(status)
    if sig == signal.SIGALRM:
      return TestOutcome(TIMEOUT, "a test exceeded its time budget", "", 0, 0.0)
    return TestOutcome(CRASHED, signal.Signals(sig).name, "", 0, 0.0)
  if not data:
    return TestOutcome(ERROR, f"test process exited with status {os.WEXITSTATUS(status)} and no result", "", 0, 0.0)
  return pickle.loads(data)


# Worker state. Set by _init_worker in each pool process so jobs only need to carry a site.
_worker_lib = None
_worker_tests = None
_worker_core_tests = None


def _init_worker(lib_path):
  global _worker_lib, _worker_tests, _worker_core_tests
  _worker_lib = lib_path
  _worker_tests = load_tests()
  _worker_core_tests = _order_core_tests(_worker_tests)


def eval_baseline(module, class_name):
  tests = [t for t in _worker_tests[module] if type(t).__name__ == class_name]
  return run_isolated(lambda: run_tests(tests, _worker_lib, -1, PROCESS_TIMEOUT_SEC), PROCESS_TIMEOUT_SEC)


def eval_mutant(site, test_timeout_sec, first_tests):
  tests = tests_for_site(site, _worker_tests, _worker_core_tests, first_tests)
  t0 = time.perf_counter()
  outcome = run_isolated(lambda: run_tests(tests, _worker_lib, site.site_id, test_timeout_sec), PROCESS_TIMEOUT_SEC)
  return MutantResult(site, outcome, time.perf_counter() - t0)


def sites_by_file(sites):
  by_file = defaultdict(list)
  for site in sites:
    by_file[site.origin_file].append(site)
  return by_file


# ***** reporting *****

def format_site_snippet(site, context_lines=2):
  lines = site.origin_file.read_text().split("\n")
  line_idx = site.origin_line - 1
  start = max(0, line_idx - context_lines)
  end = min(len(lines), line_idx + context_lines + 1)

  width = len(str(end))
  snippet = [f"{'>' if idx == line_idx else ' '} {idx + 1:>{width}} | {lines[idx]}" for idx in range(start, end)]
  # the mutated expression from the preprocessed source, so macros are expanded and the operator is unambiguous
  marker = colorize(f"[[{site.original_op}->{site.mutated_op}]]", ANSI_RED)
  snippet.append(f"  {' ' * width} > {site.snippet.replace(f'[[{site.original_op}->{site.mutated_op}]]', marker)}")
  return "\n".join(snippet)


def render_progress(counts, total, elapsed_sec):
  completed = sum(counts.values())
  bar_width = 30
  filled = int((completed / total) * bar_width)
  bar = "#" * filled + "-" * (bar_width - filled)

  rate = completed / elapsed_sec if elapsed_sec > 0 else 0.0
  eta = ((total - completed) / rate) if rate > 0 else 0.0

  colors = {KILLED: ANSI_GREEN, SURVIVED: ANSI_RED, CRASHED: ANSI_YELLOW, TIMEOUT: ANSI_YELLOW, ERROR: ANSI_YELLOW}
  tally = " ".join(colorize(f"{o[0]}:{counts[o]}", colors[o]) for o in OUTCOMES)
  return f"[{bar}] {completed}/{total} {tally} mps:{rate:.2f} elapsed:{elapsed_sec:.1f}s eta:{eta:.1f}s"


def print_live_status(text, *, final=False):
  if sys.stdout.isatty():
    print("\r" + text, end="\n" if final else "", flush=True)
  else:
    print(text, flush=True)


def load_equivalent_mutants():
  lines = (line.strip() for line in EQUIVALENT_MUTANTS.read_text().splitlines())
  return {line for line in lines if line and not line.startswith("#")}


def print_results(title, results, color, snippet):
  if not results:
    return
  print("", flush=True)
  print(colorize(title, color), flush=True)
  for res in sorted(results, key=lambda r: r.site.site_id):
    detail = " ".join(part for part in (res.outcome.test_id, res.outcome.detail.strip().splitlines()[-1] if res.outcome.detail else "") if part)
    print(f"- #{res.site.site_id} {res.site.location} [{res.site.mutator}] {format_mutation(res.site)}" + (f": {detail}" if detail else ""), flush=True)
    if snippet:
      print(format_site_snippet(res.site), flush=True)


def main():
  parser = argparse.ArgumentParser(description="Run strict safety mutation")
  parser.add_argument("-j", type=int, default=max((os.cpu_count() or 1) - 1, 1), help="parallel mutants to run")
  parser.add_argument("--max-mutants", type=int, default=0, help="optional limit for debugging (0 means all)")
  parser.add_argument("--list-only", action="store_true", help="list discovered candidates and exit")
  parser.add_argument("--filter", default="", help="only run sites whose location contains this string, e.g. honda.h or lateral.h:188")
  args = parser.parse_args()

  start = time.perf_counter()

  with tempfile.TemporaryDirectory(prefix="mutation-op-run-") as run_tmp_dir:
    preprocessed_source = preprocess(Path(run_tmp_dir) / "safety_preprocessed.c")
    sites = enumerate_sites(preprocessed_source)
    assert len(sites) > 0
    if args.filter:
      sites = [s for s in sites if args.filter in s.location]
      assert sites, f"no sites match {args.filter!r}"
    if args.max_mutants > 0:
      sites = sites[: args.max_mutants]

    mutator_counts = Counter(s.mutator for s in sites)
    mutator_summary = ", ".join(f"{m} ({mutator_counts[m]})" for m in MUTATORS if mutator_counts[m] > 0)
    print(f"Found {len(sites)} unique candidates: {mutator_summary}", flush=True)
    if args.list_only:
      for site in sites:
        print(f"  #{site.site_id:03d} {site.location} [{site.mutator}] {format_mutation(site)}")
      return 0

    discovered_count = len(sites)
    sites = [s for s in sites if not s.constexpr]
    pruned_count = discovered_count - len(sites)
    if pruned_count > 0:
      print(f"Pruned {pruned_count} build-incompatible mutants from constant-expression initializers", flush=True)
    assert sites, "all sites were pruned as build-incompatible"

    mutation_lib = Path(run_tmp_dir) / "libsafety_mutation.so"
    compile_mutated_library(preprocessed_source, sites, mutation_lib)

    # Load the tests here too: on fork platforms the workers inherit the imports
    tests = load_tests()
    print(f"Running {len(sites)} mutants with {args.j} workers", flush=True)

    with ProcessPoolExecutor(max_workers=args.j, initializer=_init_worker, initargs=(mutation_lib,)) as pool:
      # Baseline: the whole suite must pass against the instrumented library with no mutant active.
      # Otherwise a kill could come from the harness rather than the mutant. Split by class to run in parallel.
      classes = sorted({(module, type(t).__name__) for module, ts_ in tests.items() for t in ts_})
      baseline = [fut.result() for fut in as_completed([pool.submit(eval_baseline, *c) for c in classes])]
      baseline_failures = [b for b in baseline if b.outcome != SURVIVED]
      if baseline_failures:
        print("Baseline failed with no mutant active; aborting to avoid false kills.", flush=True)
        for b in baseline_failures:
          print(f"  {b.outcome}: {b.detail}", flush=True)
        return 2
      slowest_test_sec = max(b.slowest_test_sec for b in baseline)
      test_timeout_sec = max(TEST_TIMEOUT_MIN_SEC, TEST_TIMEOUT_FACTOR * slowest_test_sec)
      print(f"Baseline passed in {time.perf_counter() - start:.1f}s, per-test timeout {test_timeout_sec:.1f}s", flush=True)

      results = []
      counts = Counter({o: 0 for o in OUTCOMES})
      killers = defaultdict(Counter)  # origin file -> tests that killed its mutants
      cached_killers = json.loads(KILLER_CACHE.read_text()) if KILLER_CACHE.exists() else {}
      # Sites in shared code may need the whole suite, so start them first to avoid a long tail. Interleave files,
      # so the tests that kill a file's first mutants are known by the time the rest of that file is submitted.
      position_in_file = {site.site_id: i for file_sites in sites_by_file(sites).values() for i, site in enumerate(file_sites)}
      pending = deque(sorted(sites, key=lambda s: (s.is_mode, position_in_file[s.site_id])))
      running = set()
      print_live_status(render_progress(counts, len(sites), 0.0))
      while pending or running:
        # submit lazily, so each mutant can start with the tests that killed earlier mutants in its file
        while pending and len(running) < args.j + 2:
          site = pending.popleft()
          first_tests = [test_id for test_id, _ in killers[site.origin_file].most_common(KILLER_HINTS)]
          if site.cache_key in cached_killers:
            first_tests.insert(0, cached_killers[site.cache_key])
          running.add(pool.submit(eval_mutant, site, test_timeout_sec, first_tests))
        done, running = wait(running, return_when=FIRST_COMPLETED)
        for fut in done:
          res = fut.result()
          if res.outcome.outcome == KILLED:
            killers[res.site.origin_file][res.outcome.test_id] += 1
          results.append(res)
          counts[res.outcome.outcome] += 1
          print_live_status(render_progress(counts, len(sites), time.perf_counter() - start), final=len(results) == len(sites))

    # remember what killed each mutant; a full run also forgets mutants that no longer exist
    site_keys = {s.cache_key for s in sites}
    if not args.filter and args.max_mutants == 0:
      cached_killers = {k: v for k, v in cached_killers.items() if k in site_keys}
    cached_killers.update({r.site.cache_key: r.outcome.test_id for r in results if r.outcome.outcome == KILLED})
    cache_tmp = KILLER_CACHE.with_suffix(".tmp")
    cache_tmp.write_text(json.dumps(dict(sorted(cached_killers.items())), indent=0) + "\n")
    os.replace(cache_tmp, KILLER_CACHE)  # atomic, so concurrent runs never read a torn file

    by_outcome = {o: [r for r in results if r.outcome.outcome == o] for o in OUTCOMES}
    equivalents = load_equivalent_mutants()
    survivors = [r for r in by_outcome[SURVIVED] if r.site.cache_key not in equivalents]
    # an allowlisted mutant that got killed or no longer exists means the list is stale
    stale = [r for r in results if r.outcome.outcome != SURVIVED and r.site.cache_key in equivalents]
    if not args.filter and args.max_mutants == 0:
      stale += [k for k in equivalents if k not in site_keys]
    print_results("Surviving mutants", survivors, ANSI_RED, snippet=True)
    print_results("Crashed mutants (counted as detected)", by_outcome[CRASHED], ANSI_YELLOW, snippet=False)
    print_results("Timed out mutants (counted as detected)", by_outcome[TIMEOUT], ANSI_YELLOW, snippet=False)
    print_results("Errors", by_outcome[ERROR], ANSI_YELLOW, snippet=False)
    if stale:
      print("", flush=True)
      print(colorize(f"Stale entries in {EQUIVALENT_MUTANTS.relative_to(ROOT)} (killed or gone), remove them", ANSI_RED), flush=True)
      for entry in stale:
        print(f"- {entry.site.cache_key if isinstance(entry, MutantResult) else entry}", flush=True)

    elapsed = time.perf_counter() - start
    total_test_sec = sum(r.test_sec for r in results)
    print("", flush=True)
    print(colorize("Mutation summary", ANSI_BOLD), flush=True)
    print(f"  discovered: {discovered_count}", flush=True)
    print(f"  pruned_build_incompatible: {pruned_count}", flush=True)
    print(f"  total: {len(sites)}", flush=True)
    print(f"  {KILLED}: {colorize(str(counts[KILLED]), ANSI_GREEN)}", flush=True)
    print(f"  {SURVIVED}: {colorize(str(len(survivors)), ANSI_RED)} (+{counts[SURVIVED] - len(survivors)} known equivalent)", flush=True)
    for outcome in (CRASHED, TIMEOUT, ERROR):
      print(f"  {outcome}: {colorize(str(counts[outcome]), ANSI_YELLOW)}", flush=True)
    print(f"  test_time_sum: {total_test_sec:.2f}s", flush=True)
    print(f"  avg_test_per_mutant: {total_test_sec / len(results):.3f}s", flush=True)
    print(f"  avg_tests_run_per_mutant: {sum(r.outcome.tests_run for r in results) / len(results):.1f}", flush=True)
    print(f"  mutants_per_second: {len(sites) / elapsed:.2f}", flush=True)
    print(f"  elapsed: {elapsed:.2f}s", flush=True)

    if counts[ERROR] > 0:
      return 2
    return 1 if survivors or stale else 0


if __name__ == "__main__":
  raise SystemExit(main())
