#!/usr/bin/env python3
import argparse
import io
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import unittest
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, replace
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SAFETY_DIR = ROOT / "opendbc" / "safety"
SAFETY_TESTS_DIR = ROOT / "opendbc" / "safety" / "tests"
SAFETY_C_REL = Path("opendbc/safety/tests/libsafety/safety.c")

ANSI_RESET = "\033[0m"
ANSI_BOLD = "\033[1m"
ANSI_RED = "\033[31m"
ANSI_GREEN = "\033[32m"
ANSI_YELLOW = "\033[33m"


COMPARISON_OPERATOR_MAP = {
  "==": "!=",
  "!=": "==",
  ">": "<=",
  ">=": "<",
  "<": ">=",
  "<=": ">",
}

MUTATOR_FAMILIES = {
  "increment": ("UnaryOperator", {"++": "--"}),
  "decrement": ("UnaryOperator", {"--": "++"}),
  "comparison": ("BinaryOperator", COMPARISON_OPERATOR_MAP),
  "boundary": ("IntegerLiteral", {}),
  "bitwise_assignment": ("CompoundAssignOperator", {"&=": "|=", "|=": "&=", "^=": "&="}),
  "bitwise": ("BinaryOperator", {"&": "|", "|": "&", "^": "&"}),
  "arithmetic_assignment": ("CompoundAssignOperator", {"+=": "-=", "-=": "+=", "*=": "/=", "/=": "*=", "%=": "*="}),
  "arithmetic": ("BinaryOperator", {"+": "-", "-": "+", "*": "/", "/": "*", "%": "*"}),
  "remove_negation": ("UnaryOperator", {"!": ""}),
}


@dataclass(frozen=True)
class MutationSite:
  site_id: int
  expr_start: int
  expr_end: int
  op_start: int
  op_end: int
  line: int
  original_op: str
  mutated_op: str
  mutator: str
  origin_file: Path | None
  origin_line: int | None


@dataclass(frozen=True)
class MutantResult:
  site: MutationSite
  outcome: str  # killed | survived | infra_error
  test_sec: float
  details: str


def colorize(text, color):
  term = os.environ.get("TERM", "")
  if not sys.stdout.isatty() or term in ("", "dumb") or "NO_COLOR" in os.environ:
    return text
  return f"{color}{text}{ANSI_RESET}"


def format_mutation(original_op, mutated_op):
  return colorize(f"{original_op}->{mutated_op}", ANSI_RED)


def _spelling_loc(loc):
  if not isinstance(loc, dict):
    return None
  spelling = loc.get("spellingLoc")
  if isinstance(spelling, dict):
    return spelling
  return loc


def _make_site(txt, rule, expr_start, expr_end, op_start, op_end, line):
  mutator, original_op, mutated_op = rule
  if expr_start < 0 or expr_end < expr_start or expr_end > len(txt):
    return None
  if op_start < 0 or op_end < op_start or op_end > len(txt):
    return None
  if op_start < expr_start or op_end > expr_end:
    return None
  if txt[op_start:op_end] != original_op:
    return None

  if not isinstance(line, int):
    line = txt.count("\n", 0, op_start) + 1

  return MutationSite(
    site_id=-1,
    expr_start=expr_start,
    expr_end=expr_end,
    op_start=op_start,
    op_end=op_end,
    line=line,
    original_op=original_op,
    mutated_op=mutated_op,
    mutator=mutator,
    origin_file=None,
    origin_line=None,
  )


def _binary_like_site(node, txt, rule):
  _mutator, original_op, _mutated_op = rule
  inner = node.get("inner", [])
  if len(inner) < 2:
    return None

  begin = _spelling_loc(node.get("range", {}).get("begin", {}))
  end = _spelling_loc(node.get("range", {}).get("end", {}))
  lhs_end = _spelling_loc(inner[0].get("range", {}).get("end", {}))
  rhs_begin = _spelling_loc(inner[1].get("range", {}).get("begin", {}))
  if not isinstance(begin, dict) or not isinstance(end, dict) or not isinstance(lhs_end, dict) or not isinstance(rhs_begin, dict):
    return None

  spans = []
  lhs_end_offset = lhs_end.get("offset")
  lhs_end_tok_len = lhs_end.get("tokLen")
  rhs_begin_offset = rhs_begin.get("offset")
  if isinstance(lhs_end_offset, int) and isinstance(lhs_end_tok_len, int) and isinstance(rhs_begin_offset, int):
    spans.append((lhs_end_offset + lhs_end_tok_len, rhs_begin_offset))

  begin_offset = begin.get("offset")
  end_offset = end.get("offset")
  end_tok_len = end.get("tokLen")
  if isinstance(begin_offset, int) and isinstance(end_offset, int) and isinstance(end_tok_len, int):
    spans.append((begin_offset, end_offset + end_tok_len))

  expr_start = begin_offset if isinstance(begin_offset, int) else None
  expr_end = (end_offset + end_tok_len) if isinstance(end_offset, int) and isinstance(end_tok_len, int) else None
  if expr_start is None or expr_end is None:
    return None

  seen = set()
  for span_start, span_end in spans:
    key = (span_start, span_end)
    if key in seen or span_end < span_start or span_start < 0 or span_end > len(txt):
      continue
    seen.add(key)

    idx = txt[span_start:span_end].find(original_op)
    if idx < 0:
      continue
    op_start = span_start + idx
    op_end = op_start + len(original_op)
    site = _make_site(txt, rule, expr_start, expr_end, op_start, op_end, begin.get("line"))
    if site is not None:
      return site
  return None


def _unary_site(node, txt, rule):
  _mutator, original_op, _mutated_op = rule
  inner = node.get("inner", [])
  if not inner:
    return None

  begin = _spelling_loc(node.get("range", {}).get("begin", {}))
  end = _spelling_loc(node.get("range", {}).get("end", {}))
  operand_begin = _spelling_loc(inner[0].get("range", {}).get("begin", {}))
  operand_end = _spelling_loc(inner[0].get("range", {}).get("end", {}))
  if not isinstance(begin, dict) or not isinstance(end, dict):
    return None

  begin_offset = begin.get("offset")
  end_offset = end.get("offset")
  end_tok_len = end.get("tokLen")
  if not isinstance(begin_offset, int) or not isinstance(end_offset, int) or not isinstance(end_tok_len, int):
    return None
  expr_start = begin_offset
  expr_end = end_offset + end_tok_len

  spans = []
  postfix = node.get("isPostfix", False)
  if postfix and isinstance(operand_end, dict):
    op_end_offset = operand_end.get("offset")
    op_end_tok = operand_end.get("tokLen")
    if isinstance(op_end_offset, int) and isinstance(op_end_tok, int):
      spans.append((op_end_offset + op_end_tok, end_offset + end_tok_len))
  elif not postfix and isinstance(operand_begin, dict):
    op_begin = operand_begin.get("offset")
    if isinstance(op_begin, int):
      spans.append((begin_offset, op_begin))

  spans.append((begin_offset, end_offset + end_tok_len))

  seen = set()
  for span_start, span_end in spans:
    key = (span_start, span_end)
    if key in seen or span_end < span_start or span_start < 0 or span_end > len(txt):
      continue
    seen.add(key)

    seg = txt[span_start:span_end]
    idx = seg.rfind(original_op) if postfix else seg.find(original_op)
    if idx < 0:
      continue
    op_start = span_start + idx
    op_end = op_start + len(original_op)
    site = _make_site(txt, rule, expr_start, expr_end, op_start, op_end, begin.get("line"))
    if site is not None:
      return site
  return None


def _parse_int_literal(token):
  m = re.fullmatch(r"([0-9][0-9a-fA-FxX]*)([uUlL]*)", token)
  if m is None:
    return None
  body, suffix = m.groups()
  try:
    value = int(body, 0)
  except ValueError:
    return None
  base = "hex" if body.lower().startswith("0x") else "dec"
  return value, base, suffix


def _boundary_site(node, parent, txt):
  if parent is None:
    return None
  if parent.get("kind") != "BinaryOperator" or parent.get("opcode") not in COMPARISON_OPERATOR_MAP:
    return None

  begin = _spelling_loc(node.get("range", {}).get("begin", {}))
  end = _spelling_loc(node.get("range", {}).get("end", {}))
  if not isinstance(begin, dict) or not isinstance(end, dict):
    return None
  begin_offset = begin.get("offset")
  end_offset = end.get("offset")
  end_tok_len = end.get("tokLen")
  if not isinstance(begin_offset, int) or not isinstance(end_offset, int) or not isinstance(end_tok_len, int):
    return None

  op_start = begin_offset
  op_end = end_offset + end_tok_len
  if op_start < 0 or op_end > len(txt) or op_end <= op_start:
    return None

  token = txt[op_start:op_end]
  parsed = _parse_int_literal(token)
  if parsed is None:
    return None
  value, base, suffix = parsed
  new_value = value + 1
  if base == "hex":
    mutated = f"0x{new_value:X}{suffix}"
  else:
    mutated = f"{new_value}{suffix}"

  rule = ("boundary", token, mutated)
  return _make_site(txt, rule, op_start, op_end, op_start, op_end, begin.get("line"))


def _site_key(site):
  return (site.op_start, site.op_end, site.mutator)


def _var_decl_requires_constant_initializer(node, parent):
  if node.get("kind") != "VarDecl":
    return False
  if "init" not in node:
    return False

  storage = node.get("storageClass")
  if storage == "static":
    return True

  parent_kind = parent.get("kind") if isinstance(parent, dict) else None
  return parent_kind == "TranslationUnitDecl"


def enumerate_sites(input_source, preprocessed_file):
  subprocess.run([
    "clang",
    "-E",
    "-std=gnu11",
    "-nostdlib",
    "-fno-builtin",
    "-DALLOW_DEBUG",
    f"-I{ROOT}",
    f"-I{ROOT / 'opendbc/safety/board'}",
    str(input_source),
    "-o",
    str(preprocessed_file),
  ], cwd=ROOT, capture_output=True, check=True)

  proc = subprocess.run([
    "clang",
    "-std=gnu11",
    "-nostdlib",
    "-fno-builtin",
    "-Xclang",
    "-ast-dump=json",
    "-fsyntax-only",
    str(preprocessed_file),
  ], cwd=ROOT, check=True, text=True, capture_output=True)

  ast = json.loads(proc.stdout)
  txt = preprocessed_file.read_text()
  deduped = {}
  build_incompatible_keys = set()
  rule_map = {}
  counts = {}

  for mutator, (node_kind, op_map) in MUTATOR_FAMILIES.items():
    counts[mutator] = 0
    if mutator == "boundary":
      continue
    for original_op, mutated_op in op_map.items():
      rule = (mutator, original_op, mutated_op)
      rule_map.setdefault((node_kind, original_op), []).append(rule)

  # Build preprocessed line map
  line_map = {}
  current_map_file = None
  current_map_line = None
  directive_re = re.compile(r'^\s*#\s*(\d+)\s+"([^"]+)"')
  with preprocessed_file.open() as f:
    for pp_line_num, pp_line in enumerate(f, start=1):
      m = directive_re.match(pp_line)
      if m:
        current_map_line = int(m.group(1))
        current_map_file = Path(m.group(2)).resolve()
        continue
      if current_map_file is not None and current_map_line is not None:
        line_map[pp_line_num] = (current_map_file, current_map_line)
        current_map_line += 1

  stack = [(ast, None, False)]
  while stack:
    node, parent, in_constexpr_context = stack.pop()
    if isinstance(node, dict):
      kind_obj = node.get("kind")
      opcode_obj = node.get("opcode")
      kind = kind_obj if isinstance(kind_obj, str) else ""
      opcode = opcode_obj if isinstance(opcode_obj, str) else ""

      if kind == "IntegerLiteral":
        site = _boundary_site(node, parent, txt)
        if site is not None:
          key = _site_key(site)
          deduped[key] = site
          if in_constexpr_context:
            build_incompatible_keys.add(key)

      for rule in rule_map.get((kind, opcode), []):
        site = None
        if kind in ("BinaryOperator", "CompoundAssignOperator"):
          site = _binary_like_site(node, txt, rule)
        elif kind == "UnaryOperator":
          site = _unary_site(node, txt, rule)
        if site is not None:
          key = _site_key(site)
          deduped[key] = site
          if in_constexpr_context:
            build_incompatible_keys.add(key)

      child_constexpr = in_constexpr_context or _var_decl_requires_constant_initializer(node, parent)

      for value in node.values():
        if isinstance(value, (dict, list)):
          stack.append((value, node, child_constexpr))
    elif isinstance(node, list):
      for item in node:
        stack.append((item, parent, in_constexpr_context))

  sites = list(deduped.values())
  sites.sort(key=lambda s: (s.op_start, s.mutator))
  out = []
  build_incompatible_site_ids = set()
  for s in sites:
    mapped = line_map.get(s.line)
    if mapped is None:
      continue
    origin_file, origin_line = mapped
    if SAFETY_DIR not in origin_file.parents and origin_file != SAFETY_DIR:
      continue
    site = replace(s, site_id=len(out), origin_file=origin_file, origin_line=origin_line)
    if _site_key(site) in build_incompatible_keys:
      build_incompatible_site_ids.add(site.site_id)
    out.append(site)
    counts[site.mutator] += 1
  return out, counts, build_incompatible_site_ids


def _test_module_name(test_file):
  rel = test_file.relative_to(ROOT)
  return ".".join(rel.with_suffix("").parts)


def _tests_for_mode(stem):
  """Return test file names for a mode stem."""
  direct = f"test_{stem}.py"
  if (SAFETY_TESTS_DIR / direct).exists():
    return [direct]
  return []


def build_priority_tests(site, catalog):
  """Build an ordered list of test IDs for a mutation site.

  For mode files: all tests from matching module(s), round-robin across
  classes so the first few tests give diverse coverage with failfast.

  For core files: one test per unique method name from across all modules,
  ordered by how widely each method is shared. Methods inherited by many
  classes exercise the most fundamental safety logic and run first.
  """
  src = site.origin_file
  try:
    rel_parts = src.relative_to(ROOT).parts
  except ValueError:
    rel_parts = ()

  is_mode = len(rel_parts) >= 4 and rel_parts[:3] == ("opendbc", "safety", "modes")
  names = _tests_for_mode(src.stem) if is_mode else sorted(catalog.keys())

  if is_mode:
    # Run all tests from matching module(s) in natural order
    ordered = []
    for name in names:
      for _cls, ids in catalog.get(name, []):
        ordered.extend(ids)
    return ordered
  else:
    # Core file: multiple tests per unique method from evenly-spaced modules,
    # ordered by frequency. Different test modules initialize different safety
    # modes (torque vs angle steering, dynamic max torque, etc.), so spreading
    # across modules catches mutations that only affect certain configurations.
    MAX_PER_METHOD = 5
    method_freq = {}
    method_by_module = {}
    for name in names:
      for _cls, ids in catalog.get(name, []):
        for test_id in ids:
          method = test_id.rsplit(".", 1)[-1]
          method_freq[method] = method_freq.get(method, 0) + 1
          if method not in method_by_module:
            method_by_module[method] = {}
          if name not in method_by_module[method]:
            method_by_module[method][name] = test_id
    # Pick evenly-spaced modules for each method to maximize configuration diversity
    method_ids = {}
    for method, module_map in method_by_module.items():
      modules = sorted(module_map.keys())
      n = len(modules)
      if n <= MAX_PER_METHOD:
        method_ids[method] = [module_map[m] for m in modules]
      else:
        step = n / MAX_PER_METHOD
        method_ids[method] = [module_map[modules[int(i * step)]] for i in range(MAX_PER_METHOD)]
    # Round-robin: first instance of each method (by freq), then second, etc.
    # This ensures diverse early coverage with failfast.
    sorted_methods = sorted(method_freq, key=lambda m: -method_freq[m])
    ordered = []
    for round_idx in range(MAX_PER_METHOD):
      for m in sorted_methods:
        ids = method_ids.get(m, [])
        if round_idx < len(ids):
          ordered.append(ids[round_idx])
    return ordered


def format_site_snippet(site, context_lines=2):
  source = site.origin_file
  text = source.read_text()
  lines = text.splitlines()
  if not lines:
    return ""

  display_ln = site.origin_line
  line_idx = max(0, min(display_ln - 1, len(lines) - 1))
  start = max(0, line_idx - context_lines)
  end = min(len(lines), line_idx + context_lines + 1)

  line_text = lines[line_idx]
  rel_start = line_text.find(site.original_op)
  if rel_start < 0:
    rel_start = 0
  rel_end = rel_start + len(site.original_op)

  snippet_lines = []
  width = len(str(end))
  for idx in range(start, end):
    num = idx + 1
    prefix = ">" if idx == line_idx else " "
    line = lines[idx]
    if idx == line_idx and rel_start <= len(line):
      marker = colorize(f"[[{site.original_op}->{site.mutated_op}]]", ANSI_RED)
      line = f"{line[:rel_start]}{marker}{line[rel_end:]}"
    snippet_lines.append(f"{prefix} {num:>{width}} | {line}")
  return "\n".join(snippet_lines)


def render_progress(completed, total, killed, survived, infra, elapsed_sec):
  bar_width = 30
  filled = int((completed / total) * bar_width) if total > 0 else 0
  filled = max(0, min(bar_width, filled))
  bar = "#" * filled + "-" * (bar_width - filled)

  rate = completed / elapsed_sec if elapsed_sec > 0 else 0.0
  remaining = total - completed
  eta = (remaining / rate) if rate > 0 else 0.0

  killed_text = colorize(f"k:{killed}", ANSI_GREEN)
  survived_text = colorize(f"s:{survived}", ANSI_RED)
  infra_text = colorize(f"i:{infra}", ANSI_YELLOW)

  return f"[{bar}] {completed}/{total} {killed_text} {survived_text} {infra_text} mps:{rate:.2f} elapsed:{elapsed_sec:.1f}s eta:{eta:.1f}s"


def print_live_status(text, *, final=False):
  if sys.stdout.isatty():
    print("\r" + text, end="\n" if final else "", flush=True)
  else:
    print(text, flush=True)


def _flatten_suite(suite):
  """Recursively extract all TestCase instances from a suite."""
  tests = []
  for item in suite:
    if isinstance(item, unittest.TestSuite):
      tests.extend(_flatten_suite(item))
    else:
      tests.append(item)
  return tests


def _discover_test_catalog(lib_path):
  """Discover all test IDs grouped by class for each test file.

  Must be called in the main process before creating the worker pool so that
  forked workers inherit all imported modules (zero per-worker import cost).

  Returns: {test_file_name: [(class_name, [test_id, ...]), ...]}
  """
  from opendbc.safety.tests.libsafety import libsafety_py
  libsafety_py.load(lib_path)
  libsafety_py.libsafety.mutation_set_active_mutant(-1)

  loader = unittest.TestLoader()
  catalog = {}

  for test_file in sorted(SAFETY_TESTS_DIR.glob("test_*.py")):
    module_name = _test_module_name(test_file)
    suite = loader.loadTestsFromName(module_name)
    tests = _flatten_suite(suite)

    by_class = {}
    for t in tests:
      by_class.setdefault(type(t).__name__, []).append(t.id())

    catalog[test_file.name] = list(by_class.items())

  return catalog


def run_unittest(targets, lib_path, mutant_id, verbose):
  from opendbc.safety.tests.libsafety import libsafety_py
  libsafety_py.load(lib_path)
  libsafety_py.libsafety.mutation_set_active_mutant(mutant_id)

  if verbose:
    print("Running unittest targets:", ", ".join(targets), flush=True)

  loader = unittest.TestLoader()
  stream = io.StringIO()
  runner = unittest.TextTestRunner(stream=stream, verbosity=0, failfast=True)

  suite = unittest.TestSuite()
  for target in targets:
    suite.addTests(loader.loadTestsFromName(target))
  result = runner.run(suite)
  if result.failures:
    return result.failures[0][0].id()
  if result.errors:
    return result.errors[0][0].id()
  return None


def _instrument_source(source, sites):
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


def compile_mutated_library(preprocessed_file, sites, output_so):
  source = preprocessed_file.read_text()
  instrumented = _instrument_source(source, sites)

  prelude = """
    static int __mutation_active_id = -1;
    void mutation_set_active_mutant(int id) { __mutation_active_id = id; }
    int mutation_get_active_mutant(void) { return __mutation_active_id; }
  """
  marker_re = re.compile(r'^\s*#\s+\d+\s+"[^\n]*\n?', re.MULTILINE)
  instrumented = prelude + marker_re.sub("", instrumented)

  mutation_source = output_so.with_suffix(".c")
  mutation_source.write_text(instrumented)

  subprocess.run([
    "clang",
    "-shared",
    "-fPIC",
    "-w",
    "-fno-builtin",
    "-std=gnu11",
    "-g0",
    "-O0",
    "-DALLOW_DEBUG",
    str(mutation_source),
    "-o",
    str(output_so),
  ], cwd=ROOT, check=True)


def eval_mutant(site, targets, lib_path, verbose):
  try:
    t0 = time.perf_counter()
    failed_test = run_unittest(targets, lib_path, mutant_id=site.site_id, verbose=verbose)
    duration = time.perf_counter() - t0
    if failed_test is not None:
      return MutantResult(site, "killed", duration, "")
    return MutantResult(site, "survived", duration, "")
  except Exception as exc:
    return MutantResult(site, "infra_error", 0.0, str(exc))


def main():
  parser = argparse.ArgumentParser(description="Run strict safety mutation")
  parser.add_argument("-j", type=int, default=max((os.cpu_count() or 1) - 1, 1), help="parallel mutants to run")
  parser.add_argument("--max-mutants", type=int, default=0, help="optional limit for debugging (0 means all)")
  parser.add_argument("--list-only", action="store_true", help="list discovered candidates and exit")
  parser.add_argument("--verbose", action="store_true", help="print extra debug output")
  args = parser.parse_args()

  if args.j < 1:
    raise SystemExit("-j must be >= 1")
  if args.max_mutants < 0:
    raise SystemExit("--max-mutants must be >= 0")

  start = time.perf_counter()

  with tempfile.TemporaryDirectory(prefix="mutation-op-run-") as run_tmp_dir:
    preprocessed_file = Path(run_tmp_dir) / "safety_preprocessed.c"
    sites, mutator_counts, build_incompatible_ids = enumerate_sites(ROOT / SAFETY_C_REL, preprocessed_file)
    assert len(sites) > 0

    if args.max_mutants > 0:
      sites = sites[: args.max_mutants]

    mutator_summary = ", ".join(f"{name} ({c})" for name in MUTATOR_FAMILIES if (c := mutator_counts.get(name, 0)) > 0)
    print(f"Found {len(sites)} unique candidates: {mutator_summary}", flush=True)
    if args.list_only:
      for site in sites:
        mutation = format_mutation(site.original_op, site.mutated_op)
        print(f"  #{site.site_id:03d} {site.origin_file.relative_to(ROOT)}:{site.origin_line} [{site.mutator}] {mutation}")
      return 0

    print(f"Running {len(sites)} mutants with {args.j} workers", flush=True)

    discovered_count = len(sites)
    selected_site_ids = {s.site_id for s in sites}
    build_incompatible_ids &= selected_site_ids
    pruned_compile_sites = len(build_incompatible_ids)
    if pruned_compile_sites > 0:
      sites = [s for s in sites if s.site_id not in build_incompatible_ids]
      print(f"Pruned {pruned_compile_sites} build-incompatible mutants from constant-expression initializers", flush=True)
    if not sites:
      print("Failed to build mutation library: all sites were pruned as build-incompatible", flush=True)
      return 2

    mutation_lib = Path(run_tmp_dir) / "libsafety_mutation.so"
    compile_mutated_library(preprocessed_file, sites, mutation_lib)

    compiled_ids = {int(v) for v in re.findall(r"__mutation_active_id\s*==\s*(\d+)", mutation_lib.with_suffix(".c").read_text())}
    assert compiled_ids == {s.site_id for s in sites}, "Compiled IDs don't match runtime IDs"

    # Discover all tests by importing modules in the main process.
    # Forked workers inherit these imports, eliminating per-worker import cost.
    catalog = _discover_test_catalog(mutation_lib)

    # Baseline smoke check
    baseline_ids = [ids[0] for _cls, ids in catalog.get("test_defaults.py", []) if ids]
    baseline_failed = run_unittest(baseline_ids, mutation_lib, mutant_id=-1, verbose=args.verbose)
    if baseline_failed is not None:
      print("Baseline smoke failed with mutant_id=-1; aborting to avoid false kill signals.", flush=True)
      print(f"  failed_test: {baseline_failed}", flush=True)
      return 2

    # Pre-compute test targets per mutation site
    site_targets = {site.site_id: build_priority_tests(site, catalog) for site in sites}

    results = []
    completed = 0
    killed = 0
    survived = 0
    infra = 0

    def _record(res):
      nonlocal completed, killed, survived, infra
      results.append(res)
      completed += 1
      if res.outcome == "killed":
        killed += 1
      elif res.outcome == "survived":
        survived += 1
      else:
        infra += 1

    with ProcessPoolExecutor(max_workers=args.j) as pool:
      future_map = {
        pool.submit(eval_mutant, site, site_targets[site.site_id], mutation_lib, args.verbose): site for site in sites
      }
      print_live_status(render_progress(0, len(sites), 0, 0, 0, 0.0))
      try:
        for fut in as_completed(future_map):
          try:
            res = fut.result()
          except Exception:
            site = future_map[fut]
            res = MutantResult(site, "killed", 0.0, "worker process crashed")
          _record(res)
          elapsed_now = time.perf_counter() - start
          print_live_status(render_progress(completed, len(sites), killed, survived, infra, elapsed_now), final=(completed == len(sites)))
      except Exception:
        # Pool broken — mark all unfinished mutants as killed (crash = behavioral change detected)
        completed_ids = {r.site.site_id for r in results}
        for site in sites:
          if site.site_id not in completed_ids:
            _record(MutantResult(site, "killed", 0.0, "pool broken"))
        elapsed_now = time.perf_counter() - start
        print_live_status(render_progress(completed, len(sites), killed, survived, infra, elapsed_now), final=True)

    survivors = sorted((r for r in results if r.outcome == "survived"), key=lambda r: r.site.site_id)
    if survivors:
      print("", flush=True)
      print(colorize("Surviving mutants", ANSI_RED), flush=True)
      for res in survivors:
        loc = f"{res.site.origin_file.relative_to(ROOT)}:{res.site.origin_line}"
        mutation = format_mutation(res.site.original_op, res.site.mutated_op)
        print(f"- #{res.site.site_id} {loc} [{res.site.mutator}] {mutation}", flush=True)
        print(format_site_snippet(res.site), flush=True)

    infra_results = sorted((r for r in results if r.outcome == "infra_error"), key=lambda r: r.site.site_id)
    if infra_results:
      print("", flush=True)
      print(colorize("Infra errors", ANSI_YELLOW), flush=True)
      for res in infra_results:
        loc = f"{res.site.origin_file.relative_to(ROOT)}:{res.site.origin_line}"
        detail = res.details.splitlines()[0] if res.details else "unknown error"
        print(f"- #{res.site.site_id} {loc}: {detail}", flush=True)

    elapsed = time.perf_counter() - start
    total_test_sec = sum(r.test_sec for r in results)
    print("", flush=True)
    print(colorize("Mutation summary", ANSI_BOLD), flush=True)
    print(f"  discovered: {discovered_count}", flush=True)
    print(f"  pruned_build_incompatible: {pruned_compile_sites}", flush=True)
    print(f"  total: {len(sites)}", flush=True)
    print(f"  killed: {colorize(str(killed), ANSI_GREEN)}", flush=True)
    print(f"  survived: {colorize(str(survived), ANSI_RED)}", flush=True)
    print(f"  infra_error: {colorize(str(infra), ANSI_YELLOW)}", flush=True)
    print(f"  test_time_sum: {total_test_sec:.2f}s", flush=True)
    if results:
      print(f"  avg_test_per_mutant: {total_test_sec / len(results):.3f}s", flush=True)
    if elapsed > 0:
      print(f"  mutants_per_second: {len(sites) / elapsed:.2f}", flush=True)
    print(f"  elapsed: {elapsed:.2f}s", flush=True)

    if infra > 0:
      return 2
    if survived > 0:
      return 1
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
