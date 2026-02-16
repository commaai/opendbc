#!/usr/bin/env python3

from __future__ import annotations

import argparse
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import Future, ProcessPoolExecutor, as_completed
from dataclasses import dataclass, replace
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
SAFETY_DIR = ROOT / "opendbc" / "safety"
SAFETY_TESTS_DIR = ROOT / "opendbc" / "safety" / "tests"
SAFETY_C_REL = Path("opendbc/safety/tests/libsafety/safety.c")

SMOKE_TESTS = [
  "test_defaults.py",
  "test_elm327.py",
  "test_body.py",
]

COMPARISON_OPERATOR_MAP = {
  "==": "!=",
  "!=": "==",
  ">": "<=",
  ">=": "<",
  "<": ">=",
  "<=": ">",
}

ARITHMETIC_OPERATOR_MAP = {
  "+": "-",
  "-": "+",
  "*": "/",
  "/": "*",
  "%": "*",
}

BITWISE_OPERATOR_MAP = {
  "&": "|",
  "|": "&",
  "^": "&",
}

ARITHMETIC_ASSIGNMENT_OPERATOR_MAP = {
  "+=": "-=",
  "-=": "+=",
  "*=": "/=",
  "/=": "*=",
  "%=": "*=",
}

BITWISE_ASSIGNMENT_OPERATOR_MAP = {
  "&=": "|=",
  "|=": "&=",
  "^=": "&=",
}

INCREMENT_OPERATOR_MAP = {
  "++": "--",
}

DECREMENT_OPERATOR_MAP = {
  "--": "++",
}

REMOVE_NEGATION_OPERATOR_MAP = {
  "!": "",
}

MUTATOR_FAMILIES = {
  "comparison": ("BinaryOperator", COMPARISON_OPERATOR_MAP),
  "arithmetic": ("BinaryOperator", ARITHMETIC_OPERATOR_MAP),
  "bitwise": ("BinaryOperator", BITWISE_OPERATOR_MAP),
  "arithmetic_assignment": ("CompoundAssignOperator", ARITHMETIC_ASSIGNMENT_OPERATOR_MAP),
  "bitwise_assignment": ("CompoundAssignOperator", BITWISE_ASSIGNMENT_OPERATOR_MAP),
  "increment": ("UnaryOperator", INCREMENT_OPERATOR_MAP),
  "decrement": ("UnaryOperator", DECREMENT_OPERATOR_MAP),
  "remove_negation": ("UnaryOperator", REMOVE_NEGATION_OPERATOR_MAP),
  "boundary": ("IntegerLiteral", {}),
}

MULL_MUTATOR_FAMILIES = (
  "increment",
  "decrement",
  "comparison",
  "boundary",
  "bitwise_assignment",
  "bitwise",
  "arithmetic_assignment",
  "arithmetic",
  "remove_negation",
)


@dataclass(frozen=True)
class MutationSite:
  site_id: int
  source_file: Path
  expr_start: int
  expr_end: int
  op_start: int
  op_end: int
  line: int
  col: int
  original_op: str
  mutated_op: str
  mutator: str
  origin_file: Path | None
  origin_line: int | None


@dataclass(frozen=True)
class MutationRule:
  mutator: str
  node_kind: str
  original_op: str
  mutated_op: str


@dataclass(frozen=True)
class TestRunResult:
  returncode: int
  duration_sec: float
  failed_test: str | None
  stdout: str
  stderr: str


@dataclass(frozen=True)
class MutantResult:
  site: MutationSite
  outcome: str  # killed | survived | infra_error
  stage: str  # tests | build
  killer_test: str | None
  test_sec: float
  details: str


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Run strict safety mutation")
  parser.add_argument(
    "--mutator",
    default="all",
    choices=["all", *MULL_MUTATOR_FAMILIES],
    help="mutator family to run, or 'all' for full Mull-like set",
  )
  parser.add_argument(
    "--operator",
    default="all",
    choices=["all", *sorted(COMPARISON_OPERATOR_MAP.keys())],
    help="comparison operator filter (used only with --mutator comparison)",
  )
  parser.add_argument("-j", type=int, default=max((os.cpu_count() or 1) - 1, 1), help="parallel mutants to run")
  parser.add_argument("--max-mutants", type=int, default=0, help="optional limit for debugging (0 means all)")
  parser.add_argument("--list-only", action="store_true", help="list discovered candidates and exit")
  parser.add_argument("--verbose", action="store_true", help="print extra debug output")
  return parser.parse_args()


def find_clang() -> str:
  for clang_bin in ("clang-17", "clang"):
    if shutil.which(clang_bin):
      return clang_bin
  raise RuntimeError("clang is required (tried clang-17 and clang)")


def run_command(cmd: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None, capture: bool = False) -> subprocess.CompletedProcess[str]:
  return subprocess.run(
    cmd,
    cwd=cwd,
    env=env,
    check=False,
    text=True,
    capture_output=capture,
  )


def format_path(path: Path) -> str:
  try:
    return str(path.relative_to(ROOT))
  except ValueError:
    return str(path)


def display_file(site: MutationSite) -> Path:
  return site.origin_file if site.origin_file is not None else site.source_file


def display_line(site: MutationSite) -> int:
  return site.origin_line if site.origin_line is not None else site.line


def _spelling_loc(loc: object) -> dict | None:
  if not isinstance(loc, dict):
    return None
  spelling = loc.get("spellingLoc")
  if isinstance(spelling, dict):
    return spelling
  return loc


def _offset_to_line_col(text: str, offset: int) -> tuple[int, int]:
  line = text.count("\n", 0, offset) + 1
  line_start = text.rfind("\n", 0, offset) + 1
  col = (offset - line_start) + 1
  return line, col


def build_preprocessed_line_map(preprocessed_file: Path) -> dict[int, tuple[Path, int]]:
  mapping: dict[int, tuple[Path, int]] = {}
  current_file: Path | None = None
  current_line: int | None = None

  directive_re = re.compile(r'^\s*#\s*(\d+)\s+"([^"]+)"')
  with preprocessed_file.open() as f:
    for pp_line_num, line in enumerate(f, start=1):
      m = directive_re.match(line)
      if m:
        current_line = int(m.group(1))
        current_file = Path(m.group(2)).resolve()
        continue

      if current_file is not None and current_line is not None:
        mapping[pp_line_num] = (current_file, current_line)
        current_line += 1

  return mapping


def resolve_rules(selected_mutator: str, operator_filter: str) -> list[MutationRule]:
  mutators = list(MULL_MUTATOR_FAMILIES) if selected_mutator == "all" else [selected_mutator]
  rules: list[MutationRule] = []

  for mutator in mutators:
    node_kind, op_map = MUTATOR_FAMILIES[mutator]
    if mutator == "boundary":
      rules.append(MutationRule(mutator=mutator, node_kind=node_kind, original_op="", mutated_op=""))
      continue

    for original_op, mutated_op in op_map.items():
      if mutator == "comparison" and operator_filter != "all" and original_op != operator_filter:
        continue
      rules.append(MutationRule(mutator=mutator, node_kind=node_kind, original_op=original_op, mutated_op=mutated_op))

  return rules


def _source_text(preprocessed_file: Path, source_cache: dict[Path, str]) -> str:
  txt = source_cache.get(preprocessed_file)
  if txt is None:
    txt = preprocessed_file.read_text()
    source_cache[preprocessed_file] = txt
  return txt


def _make_site(
  preprocessed_file: Path,
  source_cache: dict[Path, str],
  rule: MutationRule,
  expr_start: int,
  expr_end: int,
  op_start: int,
  op_end: int,
  line: int | None,
  col: int | None,
) -> MutationSite | None:
  txt = _source_text(preprocessed_file, source_cache)
  if expr_start < 0 or expr_end < expr_start or expr_end > len(txt):
    return None
  if op_start < 0 or op_end < op_start or op_end > len(txt):
    return None
  if op_start < expr_start or op_end > expr_end:
    return None
  if txt[op_start:op_end] != rule.original_op:
    return None

  if not isinstance(line, int) or not isinstance(col, int):
    line, col = _offset_to_line_col(txt, op_start)

  return MutationSite(
    site_id=-1,
    source_file=preprocessed_file,
    expr_start=expr_start,
    expr_end=expr_end,
    op_start=op_start,
    op_end=op_end,
    line=line,
    col=col,
    original_op=rule.original_op,
    mutated_op=rule.mutated_op,
    mutator=rule.mutator,
    origin_file=None,
    origin_line=None,
  )


def _binary_like_site(node: dict, preprocessed_file: Path, source_cache: dict[Path, str], rule: MutationRule) -> MutationSite | None:
  inner = node.get("inner", [])
  if len(inner) < 2:
    return None

  begin = _spelling_loc(node.get("range", {}).get("begin", {}))
  end = _spelling_loc(node.get("range", {}).get("end", {}))
  lhs_end = _spelling_loc(inner[0].get("range", {}).get("end", {}))
  rhs_begin = _spelling_loc(inner[1].get("range", {}).get("begin", {}))
  if not isinstance(begin, dict) or not isinstance(end, dict) or not isinstance(lhs_end, dict) or not isinstance(rhs_begin, dict):
    return None

  spans: list[tuple[int, int]] = []
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

  txt = _source_text(preprocessed_file, source_cache)
  seen: set[tuple[int, int]] = set()
  for span_start, span_end in spans:
    key = (span_start, span_end)
    if key in seen or span_end < span_start or span_start < 0 or span_end > len(txt):
      continue
    seen.add(key)

    idx = txt[span_start:span_end].find(rule.original_op)
    if idx < 0:
      continue
    op_start = span_start + idx
    op_end = op_start + len(rule.original_op)
    if expr_start is None or expr_end is None:
      continue
    site = _make_site(preprocessed_file, source_cache, rule, expr_start, expr_end, op_start, op_end, begin.get("line"), begin.get("col"))
    if site is not None:
      return site
  return None


def _unary_site(node: dict, preprocessed_file: Path, source_cache: dict[Path, str], rule: MutationRule) -> MutationSite | None:
  inner = node.get("inner", [])
  if len(inner) < 1:
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

  spans: list[tuple[int, int]] = []
  postfix = bool(node.get("isPostfix", False))
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

  txt = _source_text(preprocessed_file, source_cache)
  seen: set[tuple[int, int]] = set()
  for span_start, span_end in spans:
    key = (span_start, span_end)
    if key in seen or span_end < span_start or span_start < 0 or span_end > len(txt):
      continue
    seen.add(key)

    seg = txt[span_start:span_end]
    idx = seg.rfind(rule.original_op) if postfix else seg.find(rule.original_op)
    if idx < 0:
      continue
    op_start = span_start + idx
    op_end = op_start + len(rule.original_op)
    site = _make_site(preprocessed_file, source_cache, rule, expr_start, expr_end, op_start, op_end, begin.get("line"), begin.get("col"))
    if site is not None:
      return site
  return None


def _parse_int_literal(token: str) -> tuple[int, str, str] | None:
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


def _boundary_site(node: dict, parent: dict | None, preprocessed_file: Path, source_cache: dict[Path, str]) -> MutationSite | None:
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

  txt = _source_text(preprocessed_file, source_cache)
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

  rule = MutationRule(mutator="boundary", node_kind="IntegerLiteral", original_op=token, mutated_op=mutated)
  return _make_site(preprocessed_file, source_cache, rule, op_start, op_end, op_start, op_end, begin.get("line"), begin.get("col"))


def preprocess_source(clang_bin: str, input_source: Path, preprocessed_source: Path) -> None:
  cmd = [
    clang_bin,
    "-E",
    "-std=gnu11",
    "-nostdlib",
    "-fno-builtin",
    "-DALLOW_DEBUG",
    f"-I{ROOT}",
    f"-I{ROOT / 'opendbc/safety/board'}",
    str(input_source),
    "-o",
    str(preprocessed_source),
  ]
  proc = run_command(cmd, cwd=ROOT, capture=True)
  if proc.returncode != 0:
    raise RuntimeError(f"Failed to preprocess source:\n{proc.stderr}")


def enumerate_sites(clang_bin: str, rules: list[MutationRule], preprocessed_file: Path) -> tuple[list[MutationSite], dict[str, int]]:
  cmd = [
    clang_bin,
    "-std=gnu11",
    "-nostdlib",
    "-fno-builtin",
    "-Xclang",
    "-ast-dump=json",
    "-fsyntax-only",
    str(preprocessed_file),
  ]
  proc = run_command(cmd, cwd=ROOT, capture=True)
  if proc.returncode != 0:
    raise RuntimeError(f"Failed to parse AST:\n{proc.stderr}")

  ast = json.loads(proc.stdout)
  source_cache: dict[Path, str] = {}
  deduped: dict[tuple[Path, int, int, str], MutationSite] = {}
  line_map = build_preprocessed_line_map(preprocessed_file)
  rule_map: dict[tuple[str, str], list[MutationRule]] = {}
  boundary_enabled = False
  counts: dict[str, int] = {}

  for rule in rules:
    counts[rule.mutator] = 0
    if rule.mutator == "boundary":
      boundary_enabled = True
      continue
    key = (rule.node_kind, rule.original_op)
    rule_map.setdefault(key, []).append(rule)

  stack: list[tuple[object, dict | None]] = [(ast, None)]
  while stack:
    node, parent = stack.pop()
    if isinstance(node, dict):
      kind_obj = node.get("kind")
      opcode_obj = node.get("opcode")
      kind = kind_obj if isinstance(kind_obj, str) else ""
      opcode = opcode_obj if isinstance(opcode_obj, str) else ""

      if boundary_enabled and kind == "IntegerLiteral":
        site = _boundary_site(node, parent, preprocessed_file, source_cache)
        if site is not None:
          deduped[(site.source_file, site.op_start, site.op_end, site.mutator)] = site

      for rule in rule_map.get((str(kind), str(opcode)), []):
        site = None
        if rule.node_kind in ("BinaryOperator", "CompoundAssignOperator"):
          site = _binary_like_site(node, preprocessed_file, source_cache, rule)
        elif rule.node_kind == "UnaryOperator":
          site = _unary_site(node, preprocessed_file, source_cache, rule)
        if site is not None:
          deduped[(site.source_file, site.op_start, site.op_end, site.mutator)] = site

      for value in node.values():
        if isinstance(value, (dict, list)):
          stack.append((value, node))
    elif isinstance(node, list):
      for item in node:
        stack.append((item, parent))

  sites = list(deduped.values())
  sites.sort(key=lambda s: (s.line, s.col, s.op_start, s.mutator))
  out: list[MutationSite] = []
  for s in sites:
    mapped = line_map.get(s.line)
    if mapped is None:
      continue
    origin_file, origin_line = mapped
    if SAFETY_DIR not in origin_file.parents and origin_file != SAFETY_DIR:
      continue
    site = replace(s, site_id=len(out), origin_file=origin_file, origin_line=origin_line)
    out.append(site)
    counts[site.mutator] = counts.get(site.mutator, 0) + 1
  return out, counts


def build_priority_tests(site: MutationSite) -> list[Path]:
  tests_by_name = {p.name: p for p in sorted(SAFETY_TESTS_DIR.glob("test_*.py"))}
  ordered_names: list[str] = []
  ordered_names.extend(SMOKE_TESTS)

  rel_parts: tuple[str, ...] = ()
  src = display_file(site)
  try:
    rel_parts = src.relative_to(ROOT).parts
  except ValueError:
    rel_parts = ()

  if len(rel_parts) >= 4 and rel_parts[:3] == ("opendbc", "safety", "modes"):
    mode_stem = src.stem
    if mode_stem == "hyundai_common":
      ordered_names.extend(["test_hyundai.py", "test_hyundai_canfd.py"])
    elif mode_stem == "volkswagen_common":
      ordered_names.extend(["test_volkswagen_mqb.py", "test_volkswagen_pq.py", "test_volkswagen_mlb.py"])
    elif mode_stem == "subaru_preglobal":
      ordered_names.extend(["test_subaru_preglobal.py", "test_subaru.py"])
    else:
      ordered_names.append(f"test_{mode_stem}.py")

  out: list[Path] = []
  seen: set[str] = set()
  for name in ordered_names:
    if name in seen:
      continue
    seen.add(name)
    test_file = tests_by_name.get(name)
    if test_file is not None:
      out.append(test_file)
  return out


def parse_failed_unittest(stdout: str) -> str | None:
  for line in stdout.splitlines():
    match = re.match(r"^(FAIL|ERROR):\s+.+\(([^)]+)\)$", line)
    if match:
      return match.group(2)
  return None


def format_site_snippet(site: MutationSite, context_lines: int = 2) -> str:
  source = display_file(site)
  text = source.read_text()
  lines = text.splitlines()
  if not lines:
    return ""

  display_ln = display_line(site)
  line_idx = max(0, min(display_ln - 1, len(lines) - 1))
  start = max(0, line_idx - context_lines)
  end = min(len(lines), line_idx + context_lines + 1)

  if source == site.source_file:
    line_start_offset = text.rfind("\n", 0, site.op_start) + 1
    line_end_offset = text.find("\n", site.op_start)
    if line_end_offset < 0:
      line_end_offset = len(text)
    rel_start = max(0, site.op_start - line_start_offset)
    rel_end = max(rel_start, min(site.op_end - line_start_offset, line_end_offset - line_start_offset))
  else:
    line_text = lines[line_idx]
    rel_start = line_text.find(site.original_op)
    if rel_start < 0:
      rel_start = 0
    rel_end = rel_start + len(site.original_op)

  snippet_lines: list[str] = []
  width = len(str(end))
  for idx in range(start, end):
    num = idx + 1
    prefix = ">" if idx == line_idx else " "
    line = lines[idx]
    if idx == line_idx and rel_start <= len(line):
      line = f"{line[:rel_start]}[[{site.original_op}->{site.mutated_op}]]{line[rel_end:]}"
    snippet_lines.append(f"{prefix} {num:>{width}} | {line}")
  return "\n".join(snippet_lines)


def render_progress(completed: int, total: int, killed: int, survived: int, infra: int, elapsed_sec: float) -> str:
  bar_width = 30
  filled = int((completed / total) * bar_width) if total > 0 else 0
  filled = max(0, min(bar_width, filled))
  bar = "#" * filled + "-" * (bar_width - filled)

  rate = completed / elapsed_sec if elapsed_sec > 0 else 0.0
  remaining = total - completed
  eta = (remaining / rate) if rate > 0 else 0.0

  return f"[{bar}] {completed}/{total} k:{killed} s:{survived} i:{infra} mps:{rate:.2f} elapsed:{elapsed_sec:.1f}s eta:{eta:.1f}s"


def render_build_progress(attempt: int, pruned: int, remaining: int, discovered: int, elapsed_sec: float, *, done: bool = False) -> str:
  bar_width = 20
  resolved = discovered - remaining
  if discovered > 0:
    pct = min(max(resolved / discovered, 0.0), 1.0)
  else:
    pct = 1.0
  if done:
    pct = 1.0
  filled = int(bar_width * pct)
  bar = "#" * filled + "-" * (bar_width - filled)
  rate = resolved / elapsed_sec if elapsed_sec > 0 else 0.0
  eta = (remaining / rate) if rate > 0 else 0.0
  return f"[build {bar}] attempts:{attempt} pruned:{pruned}/{discovered} remaining:{remaining} rps:{rate:.2f} elapsed:{elapsed_sec:.1f}s eta:{eta:.1f}s"


def print_live_status(text: str, *, final: bool = False) -> None:
  if sys.stdout.isatty():
    print("\r" + text, end="\n" if final else "", flush=True)
  else:
    print(text, flush=True)


def _parse_compile_error_locations(error_text: str) -> set[tuple[Path, int]]:
  matches = re.findall(r"([^\s:]+\.(?:h|c)):(\d+):(\d+):", error_text)
  out: set[tuple[Path, int]] = set()
  for file_str, line_str, _ in matches:
    file_path = Path(file_str)
    if not file_path.is_absolute():
      file_path = (ROOT / file_path).resolve()
    else:
      file_path = file_path.resolve()
    out.add((file_path, int(line_str)))
  return out


def _parse_compile_error_site_ids(error_text: str) -> set[int]:
  return {int(v) for v in re.findall(r"__mutation_active_id\s*==\s*(\d+)", error_text)}


def _parse_internal_site_ids(error_text: str) -> set[int]:
  return {int(v) for v in re.findall(r"site_id=(\d+)", error_text)}


def run_unittest(files: list[Path], lib_path: Path, mutant_id: int, verbose: bool) -> TestRunResult:
  os.environ["MUTATION"] = "1"
  os.environ["LIBSAFETY_PATH"] = str(lib_path)
  os.environ["MUTATION_ACTIVE_ID"] = str(mutant_id)

  to_clear: list[str] = []
  for name in list(sys.modules):
    if name.startswith("opendbc.safety.tests.test_"):
      to_clear.append(name)
  for name in to_clear:
    sys.modules.pop(name, None)

  from opendbc.safety.tests.libsafety.libsafety_py import libsafety

  set_mutant = getattr(libsafety, "mutation_set_active_mutant", None)
  if callable(set_mutant):
    set_mutant(mutant_id)

  loader = unittest.TestLoader()
  suite = unittest.TestSuite()
  module_names: list[str] = []
  for file_path in files:
    rel = file_path.relative_to(ROOT)
    module_name = ".".join(rel.with_suffix("").parts)
    module_names.append(module_name)
    suite.addTests(loader.loadTestsFromName(module_name))

  if verbose:
    print("Running unittest modules:", ", ".join(module_names), flush=True)

  stream = io.StringIO()
  runner = unittest.TextTestRunner(stream=stream, verbosity=0, failfast=True)
  t0 = time.perf_counter()
  result = runner.run(suite)
  duration = time.perf_counter() - t0

  stdout = stream.getvalue()
  stderr = ""
  returncode = 0 if result.wasSuccessful() else 1

  combined = stdout
  failed_test = parse_failed_unittest(combined)
  if failed_test is None and not result.wasSuccessful():
    failed_test = "unittest-failure"

  return TestRunResult(
    returncode=returncode,
    duration_sec=duration,
    failed_test=failed_test,
    stdout=stdout,
    stderr=stderr,
  )


def compile_mutated_library(clang_bin: str, preprocessed_file: Path, sites: list[MutationSite], output_so: Path) -> float:
  source_text = preprocessed_file.read_text()

  instrumented = source_text
  applied_edits: list[tuple[int, int, int]] = []

  def map_index(original_index: int) -> int:
    shift = 0
    for _edit_start, edit_end, delta in applied_edits:
      if edit_end <= original_index:
        shift += delta
    return original_index + shift

  for site in sorted(sites, key=lambda s: (s.expr_start, -s.expr_end, s.op_start), reverse=True):
    expr_start = map_index(site.expr_start)
    expr_end = map_index(site.expr_end)
    op_start = map_index(site.op_start)
    op_end = map_index(site.op_end)

    if expr_start < 0 or expr_end > len(instrumented) or expr_end < expr_start:
      raise RuntimeError(f"Mutation expression range drifted (site_id={site.site_id}): {format_path(site.source_file)}:{site.line}:{site.col}")
    if op_start < expr_start or op_end > expr_end:
      raise RuntimeError(f"Mutation operator range drifted (site_id={site.site_id}): {format_path(site.source_file)}:{site.line}:{site.col}")
    if instrumented[op_start:op_end] != site.original_op:
      raise RuntimeError(f"Mutation operator token drifted (site_id={site.site_id}): {format_path(site.source_file)}:{site.line}:{site.col}")

    expr_text = instrumented[expr_start:expr_end]
    rel_op_start = op_start - expr_start
    rel_op_end = op_end - expr_start
    mutated_expr = f"{expr_text[:rel_op_start]}{site.mutated_op}{expr_text[rel_op_end:]}"
    replacement = f"((__mutation_active_id == {site.site_id}) ? ({mutated_expr}) : ({expr_text}))"
    instrumented = f"{instrumented[:expr_start]}{replacement}{instrumented[expr_end:]}"
    applied_edits.append((site.expr_start, site.expr_end, len(replacement) - (site.expr_end - site.expr_start)))

  prelude = """static int __mutation_active_id = -1;
void mutation_set_active_mutant(int id) { __mutation_active_id = id; }
int mutation_get_active_mutant(void) { return __mutation_active_id; }
"""
  marker_re = re.compile(r'^\s*#\s+\d+\s+"')
  instrumented_lines = [line for line in instrumented.splitlines() if not marker_re.match(line)]
  instrumented = prelude + "\n".join(instrumented_lines) + "\n"

  mutation_source = output_so.with_suffix(".c")
  mutation_source.write_text(instrumented)

  cmd = [
    clang_bin,
    "-shared",
    "-fPIC",
    "-Wall",
    "-Wextra",
    "-Wno-error",
    "-nostdlib",
    "-fno-builtin",
    "-std=gnu11",
    "-Wno-pointer-to-int-cast",
    "-g0",
    "-O0",
    "-DALLOW_DEBUG",
    str(mutation_source),
    "-o",
    str(output_so),
  ]

  t0 = time.perf_counter()
  proc = run_command(cmd, cwd=ROOT, capture=True)
  duration = time.perf_counter() - t0
  if proc.returncode != 0:
    raise RuntimeError(proc.stderr.strip() or "unknown compile failure")
  return duration


def eval_mutant(site: MutationSite, lib_path: Path, verbose: bool) -> MutantResult:
  all_tests = sorted(SAFETY_TESTS_DIR.glob("test_*.py"))
  priority_tests = build_priority_tests(site)
  if not priority_tests:
    priority_tests = all_tests

  try:
    test_sec = 0.0
    test_result = run_unittest(priority_tests, lib_path, mutant_id=site.site_id, verbose=verbose)
    test_sec += test_result.duration_sec
    if test_result.returncode != 0 and test_result.failed_test is not None:
      return MutantResult(site, "killed", "tests", test_result.failed_test, test_sec, "")
    if test_result.returncode != 0:
      details = (test_result.stderr or test_result.stdout).strip()
      return MutantResult(site, "infra_error", "tests", None, test_sec, details)

    priority_names = {p.name for p in priority_tests}
    all_names = {p.name for p in all_tests}
    if priority_names != all_names:
      full_result = run_unittest(all_tests, lib_path, mutant_id=site.site_id, verbose=verbose)
      test_sec += full_result.duration_sec
      if full_result.returncode != 0 and full_result.failed_test is not None:
        return MutantResult(site, "killed", "tests", full_result.failed_test, test_sec, "")
      if full_result.returncode != 0:
        details = (full_result.stderr or full_result.stdout).strip()
        return MutantResult(site, "infra_error", "tests", None, test_sec, details)

    return MutantResult(site, "survived", "tests", None, test_sec, "")
  except Exception as exc:
    return MutantResult(site, "infra_error", "build", None, 0.0, str(exc))


def baseline_smoke_test(lib_path: Path, verbose: bool) -> TestRunResult:
  smoke_files = [SAFETY_TESTS_DIR / name for name in SMOKE_TESTS if (SAFETY_TESTS_DIR / name).exists()]
  if not smoke_files:
    smoke_files = sorted(SAFETY_TESTS_DIR.glob("test_*.py"))
  return run_unittest(smoke_files, lib_path, mutant_id=-1, verbose=verbose)


def main() -> int:
  args = parse_args()
  if args.j < 1:
    raise SystemExit("-j must be >= 1")
  if args.max_mutants < 0:
    raise SystemExit("--max-mutants must be >= 0")

  start = time.perf_counter()
  clang_bin = find_clang()

  with tempfile.TemporaryDirectory(prefix="mutation-op-run-") as run_tmp_dir:
    preprocessed_file = Path(run_tmp_dir) / "safety_preprocessed.c"
    print("Preprocessing safety translation unit...", flush=True)
    preprocess_source(clang_bin, ROOT / SAFETY_C_REL, preprocessed_file)

    selected_rules = resolve_rules(args.mutator, args.operator)
    mutator_label = args.mutator if args.mutator != "all" else "all Mull-like mutators"
    print(f"Discovering mutation sites for: {mutator_label}", flush=True)

    sites, mutator_counts = enumerate_sites(clang_bin, selected_rules, preprocessed_file)
    if not sites:
      print("No mutation candidates found for selected mutator configuration.", flush=True)
      return 2

    if args.max_mutants > 0:
      sites = sites[: args.max_mutants]

    mutator_summary = ", ".join(f"{name} ({mutator_counts.get(name, 0)})" for name in MULL_MUTATOR_FAMILIES if mutator_counts.get(name, 0) > 0)
    print(f"Found {len(sites)} unique candidates: {mutator_summary}", flush=True)
    if args.list_only:
      for site in sites:
        print(f"  #{site.site_id:03d} {format_path(display_file(site))}:{display_line(site)} [{site.mutator}] {site.original_op}->{site.mutated_op}")
      return 0

    print(
      f"Running {len(sites)} mutants with {args.j} workers",
      flush=True,
    )

    discovered_count = len(sites)
    mutation_lib = Path(run_tmp_dir) / "libsafety_mutation.so"
    compile_sites = sites
    pruned_compile_sites = 0
    compile_attempt = 0
    compile_phase_start = time.perf_counter()
    print_live_status(render_build_progress(compile_attempt, pruned_compile_sites, len(compile_sites), discovered_count, 0.0))
    while True:
      compile_attempt += 1
      try:
        compile_once_sec = compile_mutated_library(clang_bin, preprocessed_file, compile_sites, mutation_lib)
        print_live_status(
          render_build_progress(
            compile_attempt,
            pruned_compile_sites,
            len(compile_sites),
            discovered_count,
            time.perf_counter() - compile_phase_start,
            done=True,
          ),
          final=True,
        )
        break
      except RuntimeError as exc:
        err_text = str(exc)
        bad_site_ids = _parse_internal_site_ids(err_text) | _parse_compile_error_site_ids(err_text)
        before = len(compile_sites)
        if bad_site_ids:
          compile_sites = [s for s in compile_sites if s.site_id not in bad_site_ids]
        else:
          bad_locations = _parse_compile_error_locations(err_text)
          if not bad_locations:
            print("Failed to build mutation library:", flush=True)
            print(err_text, flush=True)
            return 2
          compile_sites = [s for s in compile_sites if (display_file(s), display_line(s)) not in bad_locations]
        removed = before - len(compile_sites)
        if removed <= 0:
          print("Failed to build mutation library:", flush=True)
          print(err_text, flush=True)
          return 2

        pruned_compile_sites += removed
        if not compile_sites:
          print("Failed to build mutation library: all sites were pruned as build-incompatible", flush=True)
          return 2
        print_live_status(
          render_build_progress(
            compile_attempt,
            pruned_compile_sites,
            len(compile_sites),
            discovered_count,
            time.perf_counter() - compile_phase_start,
          )
        )

    if pruned_compile_sites > 0:
      sites = [replace(s, site_id=i) for i, s in enumerate(compile_sites)]
      print(f"Pruned {pruned_compile_sites} build-incompatible mutants for single-library mode", flush=True)
    else:
      sites = compile_sites

    baseline_result = baseline_smoke_test(mutation_lib, args.verbose)
    if baseline_result.returncode != 0:
      print("Baseline smoke failed with mutant_id=-1; aborting to avoid false kill signals.", flush=True)
      if baseline_result.failed_test is not None:
        print(f"  failed_test: {baseline_result.failed_test}", flush=True)
      details = baseline_result.stdout.strip()
      if details:
        print(details, flush=True)
      return 2

    results: list[MutantResult] = []
    completed = 0
    killed = 0
    survived = 0
    infra = 0

    with ProcessPoolExecutor(max_workers=args.j) as pool:
      future_map: dict[Future[MutantResult], MutationSite] = {pool.submit(eval_mutant, site, mutation_lib, args.verbose): site for site in sites}
      print_live_status(render_progress(0, len(sites), 0, 0, 0, 0.0))
      for fut in as_completed(future_map):
        res = fut.result()
        results.append(res)
        completed += 1
        if res.outcome == "killed":
          killed += 1
        elif res.outcome == "survived":
          survived += 1
        else:
          infra += 1
        elapsed_now = time.perf_counter() - start
        print_live_status(render_progress(completed, len(sites), killed, survived, infra, elapsed_now), final=(completed == len(sites)))

    survivors = sorted((r for r in results if r.outcome == "survived"), key=lambda r: r.site.site_id)
    if survivors:
      print("", flush=True)
      print("Surviving mutants", flush=True)
      for res in survivors:
        loc = f"{format_path(display_file(res.site))}:{display_line(res.site)}"
        print(f"- #{res.site.site_id} {loc} [{res.site.mutator}] {res.site.original_op}->{res.site.mutated_op}", flush=True)
        print(format_site_snippet(res.site), flush=True)

    infra_results = sorted((r for r in results if r.outcome == "infra_error"), key=lambda r: r.site.site_id)
    if infra_results:
      print("", flush=True)
      print("Infra errors", flush=True)
      for res in infra_results:
        loc = f"{format_path(display_file(res.site))}:{display_line(res.site)}"
        detail = res.details.splitlines()[0] if res.details else "unknown error"
        print(f"- #{res.site.site_id} {loc} ({res.stage}): {detail}", flush=True)

    elapsed = time.perf_counter() - start
    total_build_sec = compile_once_sec
    total_test_sec = sum(r.test_sec for r in results)
    killed_by_tests = sum(1 for r in results if r.outcome == "killed" and r.stage == "tests")
    print("", flush=True)
    print("Mutation summary", flush=True)
    print(f"  mutator: {mutator_label}", flush=True)
    print(f"  discovered: {discovered_count}", flush=True)
    print(f"  pruned_build_incompatible: {pruned_compile_sites}", flush=True)
    print(f"  total: {len(sites)}", flush=True)
    print(f"  killed: {killed}", flush=True)
    print(f"  killed_by_tests: {killed_by_tests}", flush=True)
    print(f"  survived: {survived}", flush=True)
    print(f"  infra_error: {infra}", flush=True)
    print(f"  build_time_sum: {total_build_sec:.2f}s", flush=True)
    print(f"  build_once: {compile_once_sec:.2f}s", flush=True)
    print(f"  test_time_sum: {total_test_sec:.2f}s", flush=True)
    if len(results) > 0:
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
