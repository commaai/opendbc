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
import unittest
from collections.abc import Sequence
from concurrent.futures import Future, ProcessPoolExecutor, as_completed
from dataclasses import dataclass, replace
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SAFETY_DIR = ROOT / "opendbc" / "safety"
SAFETY_TESTS_DIR = ROOT / "opendbc" / "safety" / "tests"
SAFETY_C_REL = Path("opendbc/safety/tests/libsafety/safety.c")

SMOKE_TESTS = [
  "test_defaults.py",
  "test_elm327.py",
  "test_body.py",
]

CORE_KILLER_TESTS = [
  "test_chrysler.py",
  "test_ford.py",
  "test_gm.py",
  "test_hyundai.py",
  "test_rivian.py",
  "test_tesla.py",
  "test_nissan.py",
]

TEST_IDS: dict[str, tuple[str, ...]] = {
  "test_body.py": (
    "opendbc.safety.tests.test_body.TestBody.test_manually_enable_controls_allowed",
    "opendbc.safety.tests.test_body.TestBody.test_can_flasher",
  ),
  "test_chrysler.py": (
    "opendbc.safety.tests.test_chrysler.TestChryslerRamDTSafety.test_exceed_torque_sensor",
    "opendbc.safety.tests.test_chrysler.TestChryslerRamDTSafety.test_allow_engage_with_gas_pressed",
    "opendbc.safety.tests.test_chrysler.TestChryslerRamHDSafety.test_allow_engage_with_gas_pressed",
    "opendbc.safety.tests.test_chrysler.TestChryslerSafety.test_allow_engage_with_gas_pressed",
    "opendbc.safety.tests.test_chrysler.TestChryslerRamDTSafety.test_steer_safety_check",
    "opendbc.safety.tests.test_chrysler.TestChryslerRamDTSafety.test_realtime_limit_up",
  ),
  "test_defaults.py": (
    "opendbc.safety.tests.test_defaults.TestAllOutput.test_default_controls_not_allowed",
    "opendbc.safety.tests.test_defaults.TestNoOutput.test_default_controls_not_allowed",
    "opendbc.safety.tests.test_defaults.TestSilent.test_default_controls_not_allowed",
  ),
  "test_elm327.py": ("opendbc.safety.tests.test_elm327.TestElm327.test_default_controls_not_allowed",),
  "test_ford.py": (
    "opendbc.safety.tests.test_ford.TestFordCANFDLongitudinalSafety.test_curvature_rate_limits",
    "opendbc.safety.tests.test_ford.TestFordCANFDStockSafety.test_acc_buttons",
    "opendbc.safety.tests.test_ford.TestFordLongitudinalSafety.test_acc_buttons",
    "opendbc.safety.tests.test_ford.TestFordCANFDLongitudinalSafety.test_steer_allowed",
  ),
  "test_gm.py": (
    "opendbc.safety.tests.test_gm.TestGmAscmEVSafety.test_against_torque_driver",
    "opendbc.safety.tests.test_gm.TestGmAscmSafety.test_allow_engage_with_gas_pressed",
    "opendbc.safety.tests.test_gm.TestGmCameraEVSafety.test_against_torque_driver",
    "opendbc.safety.tests.test_gm.TestGmAscmEVSafety.test_steer_safety_check",
    "opendbc.safety.tests.test_gm.TestGmAscmEVSafety.test_realtime_limits",
  ),
  "test_honda.py": (
    "opendbc.safety.tests.test_honda.TestHondaBoschAltBrakeSafety.test_steer_safety_check",
    "opendbc.safety.tests.test_honda.TestHondaNidecPcmSafety.test_allow_engage_with_gas_pressed",
    "opendbc.safety.tests.test_honda.TestHondaBoschLongSafety.test_allow_engage_with_gas_pressed",
    "opendbc.safety.tests.test_honda.TestHondaBoschAltBrakeSafety.test_allow_engage_with_gas_pressed",
    "opendbc.safety.tests.test_honda.TestHondaBoschAltBrakeSafety.test_allow_user_brake_at_zero_speed",
    "opendbc.safety.tests.test_honda.TestHondaBoschLongSafety.test_brake_safety_check",
    "opendbc.safety.tests.test_honda.TestHondaNidecPcmAltSafety.test_acc_hud_safety_check",
    "opendbc.safety.tests.test_honda.TestHondaBoschLongSafety.test_gas_safety_check",
    "opendbc.safety.tests.test_honda.TestHondaNidecPcmAltSafety.test_brake_safety_check",
    "opendbc.safety.tests.test_honda.TestHondaBoschAltBrakeSafety.test_buttons",
    "opendbc.safety.tests.test_honda.TestHondaBoschLongSafety.test_rx_hook",
    "opendbc.safety.tests.test_honda.TestHondaBoschLongSafety.test_set_resume_buttons",
    "opendbc.safety.tests.test_honda.TestHondaBoschAltBrakeSafety.test_not_allow_user_brake_when_moving",
    "opendbc.safety.tests.test_honda.TestHondaBoschLongSafety.test_buttons_with_main_off",
    "opendbc.safety.tests.test_honda.TestHondaBoschAltBrakeSafety.test_enable_control_allowed_from_cruise",
    "opendbc.safety.tests.test_honda.TestHondaBoschAltBrakeSafety.test_disable_control_allowed_from_cruise",
    "opendbc.safety.tests.test_honda.TestHondaNidecPcmAltSafety.test_buttons",
    "opendbc.safety.tests.test_honda.TestHondaBoschCANFDSafety.test_allow_user_brake_at_zero_speed",
    "opendbc.safety.tests.test_honda.TestHondaBoschAltBrakeSafety.test_no_disengage_on_gas",
    "opendbc.safety.tests.test_honda.TestHondaNidecPcmAltSafety.test_honda_fwd_brake_latching",
  ),
  "test_hyundai.py": (
    "opendbc.safety.tests.test_hyundai.TestHyundaiLegacySafety.test_steer_req_bit_frames",
    "opendbc.safety.tests.test_hyundai.TestHyundaiLongitudinalSafety.test_accel_actuation_limits",
    "opendbc.safety.tests.test_hyundai.TestHyundaiLegacySafety.test_against_torque_driver",
    "opendbc.safety.tests.test_hyundai.TestHyundaiLegacySafetyEV.test_against_torque_driver",
    "opendbc.safety.tests.test_hyundai.TestHyundaiLegacySafety.test_steer_safety_check",
    "opendbc.safety.tests.test_hyundai.TestHyundaiLegacySafety.test_realtime_limits",
  ),
  "test_hyundai_canfd.py": (
    "opendbc.safety.tests.test_hyundai_canfd.TestHyundaiCanfdLFASteering_0.test_steer_req_bit_frames",
    "opendbc.safety.tests.test_hyundai_canfd.TestHyundaiCanfdLKASteeringEV.test_against_torque_driver",
    "opendbc.safety.tests.test_hyundai_canfd.TestHyundaiCanfdLFASteering.test_against_torque_driver",
    "opendbc.safety.tests.test_hyundai_canfd.TestHyundaiCanfdLFASteeringAltButtons.test_acc_cancel",
  ),
  "test_mazda.py": ("opendbc.safety.tests.test_mazda.TestMazdaSafety.test_against_torque_driver",),
  "test_nissan.py": (
    "opendbc.safety.tests.test_nissan.TestNissanLeafSafety.test_angle_cmd_when_disabled",
    "opendbc.safety.tests.test_nissan.TestNissanLeafSafety.test_acc_buttons",
    "opendbc.safety.tests.test_nissan.TestNissanSafety.test_acc_buttons",
    "opendbc.safety.tests.test_nissan.TestNissanLeafSafety.test_angle_cmd_when_enabled",
    "opendbc.safety.tests.test_nissan.TestNissanLeafSafety.test_angle_violation",
  ),
  "test_psa.py": (
    "opendbc.safety.tests.test_psa.TestPsaStockSafety.test_angle_cmd_when_disabled",
    "opendbc.safety.tests.test_psa.TestPsaStockSafety.test_allow_engage_with_gas_pressed",
  ),
  "test_rivian.py": (
    "opendbc.safety.tests.test_rivian.TestRivianLongitudinalSafety.test_against_torque_driver",
    "opendbc.safety.tests.test_rivian.TestRivianLongitudinalSafety.test_accel_actuation_limits",
    "opendbc.safety.tests.test_rivian.TestRivianStockSafety.test_accel_actuation_limits",
    "opendbc.safety.tests.test_rivian.TestRivianLongitudinalSafety.test_steer_safety_check",
    "opendbc.safety.tests.test_rivian.TestRivianLongitudinalSafety.test_realtime_limits",
  ),
  "test_subaru.py": (
    "opendbc.safety.tests.test_subaru.TestSubaruGen1LongitudinalSafety.test_steer_req_bit_frames",
    "opendbc.safety.tests.test_subaru.TestSubaruGen1LongitudinalSafety.test_against_torque_driver",
    "opendbc.safety.tests.test_subaru.TestSubaruGen2LongitudinalSafety.test_against_torque_driver",
  ),
  "test_subaru_preglobal.py": (
    "opendbc.safety.tests.test_subaru_preglobal.TestSubaruPreglobalReversedDriverTorqueSafety.test_steer_safety_check",
    "opendbc.safety.tests.test_subaru_preglobal.TestSubaruPreglobalReversedDriverTorqueSafety.test_against_torque_driver",
    "opendbc.safety.tests.test_subaru_preglobal.TestSubaruPreglobalSafety.test_against_torque_driver",
  ),
  "test_tesla.py": (
    "opendbc.safety.tests.test_tesla.TestTeslaFSD14LongitudinalSafety.test_angle_cmd_when_disabled",
    "opendbc.safety.tests.test_tesla.TestTeslaFSD14LongitudinalSafety.test_accel_actuation_limits",
    "opendbc.safety.tests.test_tesla.TestTeslaFSD14StockSafety.test_accel_actuation_limits",
    "opendbc.safety.tests.test_tesla.TestTeslaFSD14LongitudinalSafety.test_angle_cmd_when_enabled",
    "opendbc.safety.tests.test_tesla.TestTeslaFSD14LongitudinalSafety.test_steering_angle_measurements",
    "opendbc.safety.tests.test_tesla.TestTeslaFSD14LongitudinalSafety.test_lateral_jerk_limit",
    "opendbc.safety.tests.test_tesla.TestTeslaFSD14LongitudinalSafety.test_angle_violation",
    "opendbc.safety.tests.test_tesla.TestTeslaFSD14LongitudinalSafety.test_rt_limits",
  ),
  "test_toyota.py": (
    "opendbc.safety.tests.test_toyota.TestToyotaAltBrakeSafety.test_exceed_torque_sensor",
    "opendbc.safety.tests.test_toyota.TestToyotaAltBrakeSafety.test_accel_actuation_limits",
    "opendbc.safety.tests.test_toyota.TestToyotaSafetyAngle.test_accel_actuation_limits",
    "opendbc.safety.tests.test_toyota.TestToyotaAltBrakeSafety.test_realtime_limit_up",
    "opendbc.safety.tests.test_toyota.TestToyotaAltBrakeSafety.test_steer_safety_check",
  ),
  "test_volkswagen_mlb.py": (
    "opendbc.safety.tests.test_volkswagen_mlb.TestVolkswagenMlbStockSafety.test_against_torque_driver",
    "opendbc.safety.tests.test_volkswagen_mlb.TestVolkswagenMlbStockSafety.test_against_torque_driver",
  ),
  "test_volkswagen_mqb.py": (
    "opendbc.safety.tests.test_volkswagen_mqb.TestVolkswagenMqbLongSafety.test_against_torque_driver",
    "opendbc.safety.tests.test_volkswagen_mqb.TestVolkswagenMqbLongSafety.test_accel_safety_check",
    "opendbc.safety.tests.test_volkswagen_mqb.TestVolkswagenMqbStockSafety.test_against_torque_driver",
  ),
  "test_volkswagen_pq.py": (
    "opendbc.safety.tests.test_volkswagen_pq.TestVolkswagenPqLongSafety.test_torque_cmd_enable_variants",
    "opendbc.safety.tests.test_volkswagen_pq.TestVolkswagenPqLongSafety.test_accel_actuation_limits",
    "opendbc.safety.tests.test_volkswagen_pq.TestVolkswagenPqStockSafety.test_against_torque_driver",
  ),
}

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
  duration_sec: float
  failed_test: str | None
  stdout: str


@dataclass(frozen=True)
class MutantResult:
  site: MutationSite
  outcome: str  # killed | survived | infra_error
  stage: str  # tests | build
  test_sec: float
  details: str


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Run strict safety mutation")
  parser.add_argument(
    "--mutator",
    default="all",
    choices=["all", *MUTATOR_FAMILIES],
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
  mutators = list(MUTATOR_FAMILIES) if selected_mutator == "all" else [selected_mutator]
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
  if expr_start is None or expr_end is None:
    return None

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
    site = _make_site(preprocessed_file, source_cache, rule, expr_start, expr_end, op_start, op_end, begin.get("line"), begin.get("col"))
    if site is not None:
      return site
  return None


def _unary_site(node: dict, preprocessed_file: Path, source_cache: dict[Path, str], rule: MutationRule) -> MutationSite | None:
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

  spans: list[tuple[int, int]] = []
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
  proc = subprocess.run(cmd, cwd=ROOT, check=False, text=True, capture_output=True)
  if proc.returncode != 0:
    raise RuntimeError(f"Failed to preprocess source:\n{proc.stderr}")


def _site_key(site: MutationSite) -> tuple[Path, int, int, str]:
  return (site.source_file, site.op_start, site.op_end, site.mutator)


def _var_decl_requires_constant_initializer(node: dict, parent: dict | None) -> bool:
  if node.get("kind") != "VarDecl":
    return False
  if "init" not in node:
    return False

  storage = node.get("storageClass")
  if storage == "static":
    return True

  parent_kind = parent.get("kind") if isinstance(parent, dict) else None
  return parent_kind == "TranslationUnitDecl"


def enumerate_sites(clang_bin: str, rules: list[MutationRule], preprocessed_file: Path) -> tuple[list[MutationSite], dict[str, int], set[int]]:
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
  proc = subprocess.run(cmd, cwd=ROOT, check=False, text=True, capture_output=True)
  if proc.returncode != 0:
    raise RuntimeError(f"Failed to parse AST:\n{proc.stderr}")

  ast = json.loads(proc.stdout)
  source_cache: dict[Path, str] = {}
  deduped: dict[tuple[Path, int, int, str], MutationSite] = {}
  build_incompatible_keys: set[tuple[Path, int, int, str]] = set()
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

  stack: list[tuple[object, dict | None, bool]] = [(ast, None, False)]
  while stack:
    node, parent, in_constexpr_context = stack.pop()
    if isinstance(node, dict):
      kind_obj = node.get("kind")
      opcode_obj = node.get("opcode")
      kind = kind_obj if isinstance(kind_obj, str) else ""
      opcode = opcode_obj if isinstance(opcode_obj, str) else ""

      if boundary_enabled and kind == "IntegerLiteral":
        site = _boundary_site(node, parent, preprocessed_file, source_cache)
        if site is not None:
          key = _site_key(site)
          deduped[key] = site
          if in_constexpr_context:
            build_incompatible_keys.add(key)

      for rule in rule_map.get((kind, opcode), []):
        site = None
        if rule.node_kind in ("BinaryOperator", "CompoundAssignOperator"):
          site = _binary_like_site(node, preprocessed_file, source_cache, rule)
        elif rule.node_kind == "UnaryOperator":
          site = _unary_site(node, preprocessed_file, source_cache, rule)
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
  sites.sort(key=lambda s: (s.line, s.col, s.op_start, s.mutator))
  out: list[MutationSite] = []
  build_incompatible_site_ids: set[int] = set()
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


MODE_TEST_MAP = {
  "hyundai_common": ["test_hyundai.py", "test_hyundai_canfd.py"],
  "volkswagen_common": ["test_volkswagen_mqb.py", "test_volkswagen_pq.py", "test_volkswagen_mlb.py"],
  "subaru_preglobal": ["test_subaru_preglobal.py", "test_subaru.py"],
}


def build_priority_tests(site: MutationSite) -> list[str]:
  src = display_file(site)
  try:
    rel_parts = src.relative_to(ROOT).parts
  except ValueError:
    rel_parts: tuple[str, ...] = ()

  if len(rel_parts) >= 4 and rel_parts[:3] == ("opendbc", "safety", "modes") and src.stem != "defaults":
    ordered_names = MODE_TEST_MAP.get(src.stem, [f"test_{src.stem}.py"])
  elif len(rel_parts) >= 3 and rel_parts[:2] == ("opendbc", "safety"):
    ordered_names = list(CORE_KILLER_TESTS)
  else:
    ordered_names = list(SMOKE_TESTS)

  return _test_targets_from_names(ordered_names)


def _test_module_name(test_file: Path) -> str:
  rel = test_file.relative_to(ROOT)
  return ".".join(rel.with_suffix("").parts)


def _test_targets_from_names(ordered_names: list[str]) -> list[str]:
  tests_by_name = _tests_by_name()
  out: list[str] = []
  seen_names: set[str] = set()
  seen_ids: set[str] = set()
  for name in ordered_names:
    if name in seen_names:
      continue
    seen_names.add(name)

    test_file = tests_by_name.get(name)
    if test_file is None:
      continue

    targets = TEST_IDS.get(name)
    if targets is not None:
      for t in targets:
        if t not in seen_ids:
          seen_ids.add(t)
          out.append(t)
    else:
      mod = _test_module_name(test_file)
      if mod not in seen_ids:
        seen_ids.add(mod)
        out.append(mod)
  return out


def _tests_by_name() -> dict[str, Path]:
  return {p.name: p for p in sorted(SAFETY_TESTS_DIR.glob("test_*.py"))}


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


def print_live_status(text: str, *, final: bool = False) -> None:
  if sys.stdout.isatty():
    print("\r" + text, end="\n" if final else "", flush=True)
  else:
    print(text, flush=True)


def _parse_compile_error_site_ids(error_text: str) -> set[int]:
  return {int(v) for v in re.findall(r"__mutation_active_id\s*==\s*(\d+)", error_text)}


def run_unittest(targets: Sequence[Path | str], lib_path: Path, mutant_id: int, verbose: bool) -> TestRunResult:
  os.environ["LIBSAFETY_PATH"] = str(lib_path)

  from opendbc.safety.tests.libsafety.libsafety_py import libsafety

  libsafety.mutation_set_active_mutant(mutant_id)

  loader = unittest.TestLoader()
  suite = unittest.TestSuite()
  target_names: list[str] = []
  for target in targets:
    if isinstance(target, Path):
      target_name = _test_module_name(target)
    else:
      target_name = target
    target_names.append(target_name)
    suite.addTests(loader.loadTestsFromName(target_name))

  if verbose:
    print("Running unittest targets:", ", ".join(target_names), flush=True)

  stream = io.StringIO()
  runner = unittest.TextTestRunner(stream=stream, verbosity=0, failfast=True)
  t0 = time.perf_counter()
  result = runner.run(suite)
  duration = time.perf_counter() - t0

  stdout = stream.getvalue()

  failed_test: str | None = None
  if result.failures:
    failed_test = result.failures[0][0].id()
  elif result.errors:
    failed_test = result.errors[0][0].id()

  return TestRunResult(
    duration_sec=duration,
    failed_test=failed_test,
    stdout=stdout,
  )


def _instrument_source(source: str, sites: list[MutationSite]) -> str:
  """Single-pass instrumentation that handles nested mutations correctly."""
  if not sites:
    return source

  # Sort by start ascending, end descending (outermost first when same start)
  sorted_sites = sorted(sites, key=lambda s: (s.expr_start, -s.expr_end))

  # Build containment forest using a stack
  roots: list[list] = []
  stack: list[list] = []
  for site in sorted_sites:
    while stack and stack[-1][0].expr_end <= site.expr_start:
      stack.pop()
    node: list = [site, []]
    if stack:
      stack[-1][1].append(node)
    else:
      roots.append(node)
    stack.append(node)

  def build_replacement(site: MutationSite, children: list[list]) -> str:
    parts: list[str] = []
    pos = site.expr_start
    op_rel: int | None = None
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

  result_parts: list[str] = []
  pos = 0
  for site, children in roots:
    result_parts.append(source[pos : site.expr_start])
    result_parts.append(build_replacement(site, children))
    pos = site.expr_end
  result_parts.append(source[pos:])
  return "".join(result_parts)


def compile_mutated_library(clang_bin: str, preprocessed_file: Path, sites: list[MutationSite], output_so: Path) -> float:
  source = preprocessed_file.read_text()
  instrumented = _instrument_source(source, sites)

  prelude = """static int __mutation_active_id = -1;
void mutation_set_active_mutant(int id) { __mutation_active_id = id; }
int mutation_get_active_mutant(void) { return __mutation_active_id; }
"""
  marker_re = re.compile(r'^\s*#\s+\d+\s+"[^\n]*\n?', re.MULTILINE)
  instrumented = prelude + marker_re.sub("", instrumented)

  mutation_source = output_so.with_suffix(".c")
  mutation_source.write_text(instrumented)

  cmd = [
    clang_bin,
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
  ]

  t0 = time.perf_counter()
  proc = subprocess.run(cmd, cwd=ROOT, check=False, text=True, capture_output=True)
  duration = time.perf_counter() - t0
  if proc.returncode != 0:
    raise RuntimeError(proc.stderr.strip() or "unknown compile failure")
  return duration


def eval_mutant(site: MutationSite, lib_path: Path, verbose: bool) -> MutantResult:
  priority_tests = build_priority_tests(site)
  try:
    test_result = run_unittest(priority_tests, lib_path, mutant_id=site.site_id, verbose=verbose)
    if test_result.failed_test is not None:
      return MutantResult(site, "killed", "tests", test_result.duration_sec, "")
    return MutantResult(site, "survived", "tests", test_result.duration_sec, "")
  except Exception as exc:
    return MutantResult(site, "infra_error", "build", 0.0, str(exc))


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

    sites, mutator_counts, build_incompatible_ids = enumerate_sites(clang_bin, selected_rules, preprocessed_file)
    if not sites:
      print("No mutation candidates found for selected mutator configuration.", flush=True)
      return 2

    if args.max_mutants > 0:
      sites = sites[: args.max_mutants]

    mutator_summary = ", ".join(f"{name} ({c})" for name in MUTATOR_FAMILIES if (c := mutator_counts.get(name, 0)) > 0)
    print(f"Found {len(sites)} unique candidates: {mutator_summary}", flush=True)
    if args.list_only:
      for site in sites:
        print(f"  #{site.site_id:03d} {format_path(display_file(site))}:{display_line(site)} [{site.mutator}] {site.original_op}->{site.mutated_op}")
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
    print("Building mutation library in a single pass...", flush=True)
    try:
      compile_once_sec = compile_mutated_library(clang_bin, preprocessed_file, sites, mutation_lib)
    except RuntimeError as exc:
      print("Failed to build mutation library:", flush=True)
      print(str(exc), flush=True)
      return 2

    compiled_source = mutation_lib.with_suffix(".c").read_text()
    compiled_ids = _parse_compile_error_site_ids(compiled_source)
    runtime_ids = {s.site_id for s in sites}
    assert compiled_ids == runtime_ids, f"Compiled IDs don't match runtime IDs: compiled={compiled_ids} runtime={runtime_ids}"

    smoke_targets = [SAFETY_TESTS_DIR / name for name in SMOKE_TESTS]
    baseline_result = run_unittest(smoke_targets, mutation_lib, mutant_id=-1, verbose=args.verbose)
    if baseline_result.failed_test is not None:
      print("Baseline smoke failed with mutant_id=-1; aborting to avoid false kill signals.", flush=True)
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

    def _record(res: MutantResult) -> None:
      nonlocal completed, killed, survived, infra
      results.append(res)
      completed += 1
      if res.outcome == "killed":
        killed += 1
      elif res.outcome == "survived":
        survived += 1
      else:
        infra += 1

    with ProcessPoolExecutor(max_workers=args.j, max_tasks_per_child=1) as pool:
      future_map: dict[Future[MutantResult], MutationSite] = {pool.submit(eval_mutant, site, mutation_lib, args.verbose): site for site in sites}
      print_live_status(render_progress(0, len(sites), 0, 0, 0, 0.0))
      try:
        for fut in as_completed(future_map):
          try:
            res = fut.result()
          except Exception:
            site = future_map[fut]
            res = MutantResult(site, "killed", "tests", 0.0, "worker process crashed")
          _record(res)
          elapsed_now = time.perf_counter() - start
          print_live_status(render_progress(completed, len(sites), killed, survived, infra, elapsed_now), final=(completed == len(sites)))
      except Exception:
        # Pool broken — mark all unfinished mutants as killed (crash = behavioral change detected)
        completed_ids = {r.site.site_id for r in results}
        for site in sites:
          if site.site_id not in completed_ids:
            _record(MutantResult(site, "killed", "tests", 0.0, "pool broken"))
        elapsed_now = time.perf_counter() - start
        print_live_status(render_progress(completed, len(sites), killed, survived, infra, elapsed_now), final=True)

    # Verification pass: re-run survivors to detect flaky mutants
    initial_survivors = [r for r in results if r.outcome == "survived"]
    if initial_survivors:
      print(f"\nVerifying {len(initial_survivors)} survivors...", flush=True)
      with ProcessPoolExecutor(max_workers=args.j, max_tasks_per_child=1) as pool:
        verify_futures = {pool.submit(eval_mutant, r.site, mutation_lib, args.verbose): r for r in initial_survivors}
        verified: dict[int, MutantResult] = {}
        for fut in as_completed(verify_futures):
          try:
            vres = fut.result()
          except Exception:
            orig = verify_futures[fut]
            vres = MutantResult(orig.site, "survived", "tests", 0.0, "verification crash")
          verified[vres.site.site_id] = vres
      newly_killed = {sid: v for sid, v in verified.items() if v.outcome == "killed"}
      if newly_killed:
        print(f"  {len(newly_killed)} flaky mutants reclassified as killed", flush=True)
        results = [newly_killed.get(r.site.site_id, r) if r.outcome == "survived" else r for r in results]
        killed = sum(1 for r in results if r.outcome == "killed")
        survived = sum(1 for r in results if r.outcome == "survived")
        infra = sum(1 for r in results if r.outcome == "infra_error")
        elapsed_now = time.perf_counter() - start
        print_live_status(render_progress(completed, len(sites), killed, survived, infra, elapsed_now), final=True)

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
    total_test_sec = sum(r.test_sec for r in results)
    print("", flush=True)
    print("Mutation summary", flush=True)
    print(f"  mutator: {mutator_label}", flush=True)
    print(f"  discovered: {discovered_count}", flush=True)
    print(f"  pruned_build_incompatible: {pruned_compile_sites}", flush=True)
    print(f"  total: {len(sites)}", flush=True)
    print(f"  killed: {killed}", flush=True)
    print(f"  survived: {survived}", flush=True)
    print(f"  infra_error: {infra}", flush=True)
    print(f"  build_time: {compile_once_sec:.2f}s", flush=True)
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
