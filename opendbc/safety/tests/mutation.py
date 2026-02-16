#!/usr/bin/env python3
from __future__ import annotations

import argparse, io, json, os, re, shutil, subprocess, sys, tempfile, time, unittest
from concurrent.futures import Future, ProcessPoolExecutor, as_completed
from dataclasses import dataclass, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SAFETY_DIR = ROOT / "opendbc" / "safety"
TESTS_DIR = SAFETY_DIR / "tests"
SAFETY_C = Path("opendbc/safety/tests/libsafety/safety.c")

CMP_OPS = {"==": "!=", "!=": "==", ">": "<=", ">=": "<", "<": ">=", "<=": ">"}

MUTATORS: dict[str, tuple[str, dict[str, str]]] = {
  "increment":             ("UnaryOperator",          {"++": "--"}),
  "decrement":             ("UnaryOperator",          {"--": "++"}),
  "comparison":            ("BinaryOperator",         CMP_OPS),
  "boundary":              ("IntegerLiteral",         {}),
  "bitwise_assignment":    ("CompoundAssignOperator", {"&=": "|=", "|=": "&=", "^=": "&="}),
  "bitwise":               ("BinaryOperator",         {"&": "|", "|": "&", "^": "&"}),
  "arithmetic_assignment": ("CompoundAssignOperator", {"+=": "-=", "-=": "+=", "*=": "/=", "/=": "*=", "%=": "*="}),
  "arithmetic":            ("BinaryOperator",         {"+": "-", "-": "+", "*": "/", "/": "*", "%": "*"}),
  "remove_negation":       ("UnaryOperator",          {"!": ""}),
}

# Map mode file stem -> test files; default is test_{stem}.py
MODE_TESTS: dict[str, list[str]] = {
  "hyundai_common": ["test_hyundai.py", "test_hyundai_canfd.py"],
  "volkswagen_common": ["test_volkswagen_mqb.py", "test_volkswagen_pq.py", "test_volkswagen_mlb.py"],
  "subaru_preglobal": ["test_subaru_preglobal.py", "test_subaru.py"],
}

CORE_TESTS = ["test_chrysler.py", "test_ford.py", "test_gm.py", "test_hyundai.py",
              "test_rivian.py", "test_tesla.py", "test_nissan.py"]
SMOKE_TESTS = ["test_defaults.py", "test_elm327.py", "test_body.py"]

# Compact killer tests: "Class.method" auto-expands with test file module prefix.
# Use "module::Class.method" for cross-module references.
_KILLER_TESTS: dict[str, tuple[str, ...]] = {
  "test_body.py": ("TestBody.test_manually_enable_controls_allowed", "TestBody.test_can_flasher"),
  "test_chrysler.py": (
    "TestChryslerRamDTSafety.test_exceed_torque_sensor", "TestChryslerRamDTSafety.test_allow_engage_with_gas_pressed",
    "TestChryslerRamHDSafety.test_allow_engage_with_gas_pressed", "TestChryslerSafety.test_allow_engage_with_gas_pressed",
    "TestChryslerRamDTSafety.test_steer_safety_check", "TestChryslerRamDTSafety.test_realtime_limit_up"),
  "test_defaults.py": (
    "TestAllOutput.test_default_controls_not_allowed", "TestNoOutput.test_default_controls_not_allowed",
    "TestSilent.test_default_controls_not_allowed"),
  "test_elm327.py": ("TestElm327.test_default_controls_not_allowed",),
  "test_ford.py": (
    "TestFordCANFDLongitudinalSafety.test_curvature_rate_limits", "TestFordCANFDStockSafety.test_acc_buttons",
    "TestFordLongitudinalSafety.test_acc_buttons", "TestFordCANFDLongitudinalSafety.test_steer_allowed"),
  "test_gm.py": (
    "TestGmAscmEVSafety.test_against_torque_driver", "GmLongitudinalBase.test_allow_engage_with_gas_pressed",
    "TestGmCameraEVSafety.test_against_torque_driver", "TestGmAscmEVSafety.test_steer_safety_check",
    "TestGmAscmEVSafety.test_realtime_limits"),
  "test_honda.py": (
    "TestHondaBoschAltBrakeSafety.test_steer_safety_check", "HondaBase.test_allow_engage_with_gas_pressed",
    "HondaButtonEnableBase.test_allow_engage_with_gas_pressed", "TestHondaBoschAltBrakeSafety.test_allow_engage_with_gas_pressed",
    "TestHondaBoschAltBrakeSafety.test_allow_user_brake_at_zero_speed", "TestHondaBoschLongSafety.test_brake_safety_check",
    "TestHondaNidecPcmAltSafety.test_acc_hud_safety_check", "TestHondaBoschLongSafety.test_gas_safety_check",
    "TestHondaNidecPcmAltSafety.test_brake_safety_check", "TestHondaBoschAltBrakeSafety.test_buttons",
    "TestHondaBoschLongSafety.test_rx_hook", "TestHondaBoschLongSafety.test_set_resume_buttons",
    "TestHondaBoschAltBrakeSafety.test_not_allow_user_brake_when_moving", "TestHondaBoschLongSafety.test_buttons_with_main_off",
    "TestHondaBoschAltBrakeSafety.test_enable_control_allowed_from_cruise",
    "TestHondaBoschAltBrakeSafety.test_disable_control_allowed_from_cruise",
    "TestHondaNidecPcmAltSafety.test_buttons", "TestHondaBoschCANFDSafety.test_allow_user_brake_at_zero_speed",
    "TestHondaBoschAltBrakeSafety.test_no_disengage_on_gas", "TestHondaNidecPcmAltSafety.test_honda_fwd_brake_latching"),
  "test_hyundai.py": (
    "TestHyundaiLegacySafety.test_steer_req_bit_frames",
    "hyundai_common::HyundaiLongitudinalBase.test_accel_actuation_limits",
    "TestHyundaiLegacySafety.test_against_torque_driver", "TestHyundaiLegacySafetyEV.test_against_torque_driver",
    "TestHyundaiLegacySafety.test_steer_safety_check", "TestHyundaiLegacySafety.test_realtime_limits"),
  "test_hyundai_canfd.py": (
    "TestHyundaiCanfdLFASteering_0.test_steer_req_bit_frames", "TestHyundaiCanfdBase.test_against_torque_driver",
    "TestHyundaiCanfdLFASteering.test_against_torque_driver", "TestHyundaiCanfdLFASteeringAltButtons.test_acc_cancel"),
  "test_mazda.py": ("TestMazdaSafety.test_against_torque_driver",),
  "test_nissan.py": (
    "TestNissanLeafSafety.test_angle_cmd_when_disabled", "TestNissanLeafSafety.test_acc_buttons",
    "TestNissanSafety.test_acc_buttons", "TestNissanLeafSafety.test_angle_cmd_when_enabled",
    "TestNissanLeafSafety.test_angle_violation"),
  "test_psa.py": ("TestPsaStockSafety.test_angle_cmd_when_disabled", "TestPsaSafetyBase.test_allow_engage_with_gas_pressed"),
  "test_rivian.py": (
    "TestRivianLongitudinalSafety.test_against_torque_driver", "TestRivianLongitudinalSafety.test_accel_actuation_limits",
    "TestRivianSafetyBase.test_accel_actuation_limits", "TestRivianLongitudinalSafety.test_steer_safety_check",
    "TestRivianLongitudinalSafety.test_realtime_limits"),
  "test_subaru.py": (
    "TestSubaruGen1LongitudinalSafety.test_steer_req_bit_frames",
    "TestSubaruGen1LongitudinalSafety.test_against_torque_driver",
    "TestSubaruGen2LongitudinalSafety.test_against_torque_driver"),
  "test_subaru_preglobal.py": (
    "TestSubaruPreglobalReversedDriverTorqueSafety.test_steer_safety_check",
    "TestSubaruPreglobalReversedDriverTorqueSafety.test_against_torque_driver",
    "TestSubaruPreglobalSafety.test_against_torque_driver"),
  "test_tesla.py": (
    "TestTeslaFSD14LongitudinalSafety.test_angle_cmd_when_disabled",
    "TestTeslaFSD14LongitudinalSafety.test_accel_actuation_limits",
    "TestTeslaFSD14StockSafety.test_accel_actuation_limits",
    "TestTeslaFSD14LongitudinalSafety.test_angle_cmd_when_enabled",
    "TestTeslaFSD14LongitudinalSafety.test_steering_angle_measurements",
    "TestTeslaFSD14LongitudinalSafety.test_lateral_jerk_limit",
    "TestTeslaFSD14LongitudinalSafety.test_angle_violation", "TestTeslaFSD14LongitudinalSafety.test_rt_limits"),
  "test_toyota.py": (
    "TestToyotaAltBrakeSafety.test_exceed_torque_sensor", "TestToyotaAltBrakeSafety.test_accel_actuation_limits",
    "TestToyotaSafetyAngle.test_accel_actuation_limits", "TestToyotaAltBrakeSafety.test_realtime_limit_up",
    "TestToyotaAltBrakeSafety.test_steer_safety_check"),
  "test_volkswagen_mlb.py": (
    "TestVolkswagenMlbStockSafety.test_against_torque_driver",
    "TestVolkswagenMlbSafetyBase.test_against_torque_driver"),
  "test_volkswagen_mqb.py": (
    "TestVolkswagenMqbLongSafety.test_against_torque_driver", "TestVolkswagenMqbLongSafety.test_accel_safety_check",
    "TestVolkswagenMqbStockSafety.test_against_torque_driver"),
  "test_volkswagen_pq.py": (
    "TestVolkswagenPqLongSafety.test_torque_cmd_enable_variants", "TestVolkswagenPqLongSafety.test_accel_actuation_limits",
    "TestVolkswagenPqSafetyBase.test_against_torque_driver"),
}


def _expand_test_id(file_stem: str, compact: str) -> str:
  if "::" in compact:
    module, rest = compact.split("::", 1)
    return f"opendbc.safety.tests.{module}.{rest}"
  return f"opendbc.safety.tests.{file_stem}.{compact}"


@dataclass(frozen=True)
class Site:
  id: int
  source: Path
  expr_start: int
  expr_end: int
  op_start: int
  op_end: int
  line: int
  col: int
  original: str
  replacement: str
  mutator: str
  origin_file: Path | None = None
  origin_line: int | None = None


@dataclass(frozen=True)
class Result:
  site: Site
  outcome: str       # killed | survived | infra_error
  stage: str         # tests | build
  killer: str | None
  test_sec: float
  details: str = ""


def preprocess(clang: str, src: Path, out: Path) -> None:
  r = subprocess.run(
    [clang, "-E", "-std=gnu11", "-nostdlib", "-fno-builtin", "-DALLOW_DEBUG",
     f"-I{ROOT}", f"-I{ROOT / 'opendbc/safety/board'}", str(src), "-o", str(out)],
    cwd=ROOT, capture_output=True, text=True, check=False)
  if r.returncode:
    raise RuntimeError(f"Preprocessing failed:\n{r.stderr}")


def build_line_map(pp_file: Path) -> dict[int, tuple[Path, int]]:
  mapping: dict[int, tuple[Path, int]] = {}
  cur_file: Path | None = None
  cur_line: int | None = None
  directive = re.compile(r'^\s*#\s*(\d+)\s+"([^"]+)"')
  with pp_file.open() as f:
    for pp_num, line in enumerate(f, 1):
      m = directive.match(line)
      if m:
        cur_line, cur_file = int(m.group(1)), Path(m.group(2)).resolve()
      elif cur_file is not None and cur_line is not None:
        mapping[pp_num] = (cur_file, cur_line)
        cur_line += 1
  return mapping


def _spelling(loc: object) -> dict | None:
  if not isinstance(loc, dict):
    return None
  return loc.get("spellingLoc") if isinstance(loc.get("spellingLoc"), dict) else loc


def _range_locs(node: dict) -> tuple[dict, dict] | None:
  begin = _spelling(node.get("range", {}).get("begin"))
  end = _spelling(node.get("range", {}).get("end"))
  return (begin, end) if isinstance(begin, dict) and isinstance(end, dict) else None


def _read_cached(pp_file: Path, cache: dict[Path, str]) -> str:
  if pp_file not in cache:
    cache[pp_file] = pp_file.read_text()
  return cache[pp_file]


def _make_site(pp_file: Path, cache: dict[Path, str], mutator: str,
               es: int, ee: int, os_: int, oe: int,
               orig: str, repl: str, line: int | None, col: int | None) -> Site | None:
  txt = _read_cached(pp_file, cache)
  if es < 0 or ee > len(txt) or os_ < es or oe > ee or txt[os_:oe] != orig:
    return None
  if not isinstance(line, int) or not isinstance(col, int):
    line = txt.count("\n", 0, os_) + 1
    col = os_ - txt.rfind("\n", 0, os_)
  return Site(-1, pp_file, es, ee, os_, oe, line, col, orig, repl, mutator)


def _extract_site(node: dict, pp_file: Path, cache: dict[Path, str],
                  mutator: str, orig_op: str, repl_op: str) -> Site | None:
  rng = _range_locs(node)
  if rng is None:
    return None
  begin, end = rng
  bo, eo, etl = begin.get("offset"), end.get("offset"), end.get("tokLen")
  if not all(isinstance(v, int) for v in (bo, eo, etl)):
    return None

  inner = node.get("inner", [])
  kind = node.get("kind", "")
  spans: list[tuple[int, int]] = []
  postfix = False

  if kind in ("BinaryOperator", "CompoundAssignOperator"):
    if len(inner) < 2:
      return None
    lhs_end = _spelling(inner[0].get("range", {}).get("end"))
    rhs_begin = _spelling(inner[1].get("range", {}).get("begin"))
    if not isinstance(lhs_end, dict) or not isinstance(rhs_begin, dict):
      return None
    le, letl, rb = lhs_end.get("offset"), lhs_end.get("tokLen"), rhs_begin.get("offset")
    if not all(isinstance(v, int) for v in (le, letl, rb)):
      return None
    spans = [(le + letl, rb), (bo, eo + etl)]
  elif kind == "UnaryOperator":
    if not inner:
      return None
    postfix = bool(node.get("isPostfix"))
    if postfix:
      loc = _spelling(inner[0].get("range", {}).get("end"))
      if isinstance(loc, dict):
        o, t = loc.get("offset"), loc.get("tokLen")
        if isinstance(o, int) and isinstance(t, int):
          spans.append((o + t, eo + etl))
    else:
      loc = _spelling(inner[0].get("range", {}).get("begin"))
      if isinstance(loc, dict) and isinstance(loc.get("offset"), int):
        spans.append((bo, loc["offset"]))
    spans.append((bo, eo + etl))

  txt = _read_cached(pp_file, cache)
  pos = None
  for start, end_ in spans:
    if start < 0 or end_ > len(txt) or end_ < start:
      continue
    seg = txt[start:end_]
    idx = seg.rfind(orig_op) if postfix else seg.find(orig_op)
    if idx >= 0:
      pos = start + idx
      break
  if pos is None:
    return None
  return _make_site(pp_file, cache, mutator, bo, eo + etl, pos, pos + len(orig_op),
                    orig_op, repl_op, begin.get("line"), begin.get("col"))


def _boundary_site(node: dict, parent: dict | None, pp_file: Path,
                   cache: dict[Path, str]) -> Site | None:
  if not parent or parent.get("kind") != "BinaryOperator" or parent.get("opcode") not in CMP_OPS:
    return None
  rng = _range_locs(node)
  if rng is None:
    return None
  begin, end = rng
  bo, eo, etl = begin.get("offset"), end.get("offset"), end.get("tokLen")
  if not all(isinstance(v, int) for v in (bo, eo, etl)):
    return None
  txt = _read_cached(pp_file, cache)
  token = txt[bo:eo + etl]
  m = re.fullmatch(r"([0-9][0-9a-fA-FxX]*)([uUlL]*)", token)
  if m is None:
    return None
  body, suffix = m.groups()
  try:
    value = int(body, 0)
  except ValueError:
    return None
  mutated = f"0x{value + 1:X}{suffix}" if body.lower().startswith("0x") else f"{value + 1}{suffix}"
  return _make_site(pp_file, cache, "boundary", bo, eo + etl, bo, eo + etl,
                    token, mutated, begin.get("line"), begin.get("col"))


def discover_sites(clang: str, pp_file: Path, mutator_filter: str) -> tuple[list[Site], dict[str, int]]:
  r = subprocess.run(
    [clang, "-std=gnu11", "-nostdlib", "-fno-builtin", "-Xclang",
     "-ast-dump=json", "-fsyntax-only", str(pp_file)],
    cwd=ROOT, capture_output=True, text=True, check=False)
  if r.returncode:
    raise RuntimeError(f"AST parse failed:\n{r.stderr}")

  ast = json.loads(r.stdout)
  cache: dict[Path, str] = {}
  deduped: dict[tuple[Path, int, int, str], Site] = {}
  line_map = build_line_map(pp_file)

  active = list(MUTATORS) if mutator_filter == "all" else [mutator_filter]
  rules: dict[tuple[str, str], list[tuple[str, str, str]]] = {}
  has_boundary = False
  counts: dict[str, int] = {m: 0 for m in active}
  for name in active:
    kind, ops = MUTATORS[name]
    if name == "boundary":
      has_boundary = True
      continue
    for orig, repl in ops.items():
      rules.setdefault((kind, orig), []).append((name, orig, repl))

  stack: list[tuple[object, dict | None]] = [(ast, None)]
  while stack:
    node, parent = stack.pop()
    if isinstance(node, dict):
      kind = node.get("kind", "")
      opcode = node.get("opcode", "")
      if has_boundary and kind == "IntegerLiteral":
        s = _boundary_site(node, parent, pp_file, cache)
        if s:
          deduped[(s.source, s.op_start, s.op_end, s.mutator)] = s
      for name, orig, repl in rules.get((kind, opcode), []):
        s = _extract_site(node, pp_file, cache, name, orig, repl)
        if s:
          deduped[(s.source, s.op_start, s.op_end, s.mutator)] = s
      for v in node.values():
        if isinstance(v, (dict, list)):
          stack.append((v, node))
    elif isinstance(node, list):
      for item in node:
        stack.append((item, parent))

  sites = sorted(deduped.values(), key=lambda s: (s.line, s.col, s.op_start, s.mutator))
  out: list[Site] = []
  for s in sites:
    mapped = line_map.get(s.line)
    if mapped is None:
      continue
    origin_file, origin_line = mapped
    if SAFETY_DIR not in origin_file.parents and origin_file != SAFETY_DIR:
      continue
    site = replace(s, id=len(out), origin_file=origin_file, origin_line=origin_line)
    out.append(site)
    counts[site.mutator] = counts.get(site.mutator, 0) + 1
  return out, counts


def compile_library(clang: str, pp_file: Path, sites: list[Site], output: Path) -> float:
  source = pp_file.read_text()
  instrumented = source
  edits: list[tuple[int, int, int]] = []

  def shift(idx: int) -> int:
    return idx + sum(d for _, e, d in edits if e <= idx)

  for site in sorted(sites, key=lambda s: (s.expr_start, -s.expr_end, s.op_start), reverse=True):
    es, ee = shift(site.expr_start), shift(site.expr_end)
    os_, oe = shift(site.op_start), shift(site.op_end)
    if es < 0 or ee > len(instrumented) or os_ < es or oe > ee:
      raise RuntimeError(f"Range drifted (site_id={site.id})")
    if instrumented[os_:oe] != site.original:
      raise RuntimeError(f"Token drifted (site_id={site.id})")
    expr = instrumented[es:ee]
    ro = os_ - es
    mutated = f"{expr[:ro]}{site.replacement}{expr[ro + (oe - os_):]}"
    repl = f"((__mutation_active_id == {site.id}) ? ({mutated}) : ({expr}))"
    instrumented = f"{instrumented[:es]}{repl}{instrumented[ee:]}"
    edits.append((site.expr_start, site.expr_end, len(repl) - (site.expr_end - site.expr_start)))

  prelude = ("static int __mutation_active_id = -1;\n"
             "void mutation_set_active_mutant(int id) { __mutation_active_id = id; }\n"
             "int mutation_get_active_mutant(void) { return __mutation_active_id; }\n")
  marker = re.compile(r'^\s*#\s+\d+\s+"')
  instrumented = prelude + "\n".join(l for l in instrumented.splitlines() if not marker.match(l)) + "\n"

  src_file = output.with_suffix(".c")
  src_file.write_text(instrumented)
  t0 = time.perf_counter()
  r = subprocess.run(
    [clang, "-shared", "-fPIC", "-Wall", "-Wextra", "-Wno-error", "-nostdlib",
     "-fno-builtin", "-std=gnu11", "-Wno-pointer-to-int-cast", "-g0", "-O0",
     "-DALLOW_DEBUG", str(src_file), "-o", str(output)],
    cwd=ROOT, capture_output=True, text=True, check=False)
  dur = time.perf_counter() - t0
  if r.returncode:
    raise RuntimeError(r.stderr.strip() or "compile failed")
  return dur


def _test_module(path: Path) -> str:
  return ".".join(path.relative_to(ROOT).with_suffix("").parts)


def test_targets(site: Site) -> list[str]:
  src = site.origin_file or site.source
  try:
    rel = src.relative_to(ROOT)
  except ValueError:
    return [_test_module(p) for p in sorted(TESTS_DIR.glob("test_*.py"))]

  parts = rel.parts
  if len(parts) >= 4 and parts[:3] == ("opendbc", "safety", "modes") and rel.stem != "defaults":
    names = MODE_TESTS.get(rel.stem, [f"test_{rel.stem}.py"])
  elif len(parts) >= 3 and parts[:2] == ("opendbc", "safety"):
    names = list(CORE_TESTS)
  else:
    names = list(SMOKE_TESTS)

  out: list[str] = []
  for name in names:
    if not (TESTS_DIR / name).exists():
      continue
    targets = _KILLER_TESTS.get(name)
    if targets:
      stem = name.removesuffix(".py")
      out.extend(_expand_test_id(stem, t) for t in targets)
    else:
      out.append(_test_module(TESTS_DIR / name))
  return out


def run_tests(targets: list[str], lib: Path, mutant_id: int) -> tuple[bool, str | None, float]:
  os.environ["MUTATION"] = "1"
  os.environ["LIBSAFETY_PATH"] = str(lib)
  os.environ["MUTATION_ACTIVE_ID"] = str(mutant_id)

  from opendbc.safety.tests.libsafety.libsafety_py import libsafety
  libsafety.init_tests()
  libsafety.mutation_set_active_mutant(mutant_id)

  loader = unittest.TestLoader()
  suite = unittest.TestSuite()
  for t in targets:
    suite.addTests(loader.loadTestsFromName(t))

  stream = io.StringIO()
  runner = unittest.TextTestRunner(stream=stream, verbosity=0, failfast=True)
  t0 = time.perf_counter()
  result = runner.run(suite)
  dur = time.perf_counter() - t0

  if result.wasSuccessful():
    return False, None, dur
  failed = result.failures[0][0].id() if result.failures else \
           result.errors[0][0].id() if result.errors else "unknown"
  return True, failed, dur


def eval_mutant(site: Site, lib: Path) -> Result:
  try:
    killed, killer, dur = run_tests(test_targets(site), lib, site.id)
    return Result(site, "killed" if killed else "survived", "tests", killer, dur)
  except Exception as e:
    return Result(site, "infra_error", "build", None, 0.0, str(e))


def _site_loc(site: Site) -> str:
  return f"{(site.origin_file or site.source).relative_to(ROOT)}:{site.origin_line or site.line}"


def snippet(site: Site, ctx: int = 2) -> str:
  src = site.origin_file or site.source
  ln = site.origin_line or site.line
  if not src.exists(): return ""
  lines = src.read_text().splitlines()
  idx = max(0, min(ln - 1, len(lines) - 1))
  start, end = max(0, idx - ctx), min(len(lines), idx + ctx + 1)
  w = len(str(end))
  out = []
  for i in range(start, end):
    pfx = ">" if i == idx else " "
    line = lines[i]
    if i == idx and (p := line.find(site.original)) >= 0:
      line = f"{line[:p]}[[{site.original}->{site.replacement}]]{line[p + len(site.original):]}"
    out.append(f"{pfx} {i + 1:>{w}} | {line}")
  return "\n".join(out)


def progress(done: int, total: int, k: int, s: int, i: int, elapsed: float) -> str:
  filled = int(done / total * 30) if total else 0
  bar = "#" * filled + "-" * (30 - filled)
  rate = done / elapsed if elapsed > 0 else 0
  eta = (total - done) / rate if rate > 0 else 0
  return f"[{bar}] {done}/{total} k:{k} s:{s} i:{i} {rate:.1f}mps {elapsed:.0f}s eta:{eta:.0f}s"


def _parse_errors(text: str) -> tuple[set[int], set[tuple[Path, int]]]:
  ids = ({int(v) for v in re.findall(r"__mutation_active_id\s*==\s*(\d+)", text)} |
         {int(v) for v in re.findall(r"site_id=(\d+)", text)})
  locs: set[tuple[Path, int]] = set()
  if not ids:
    for f, ln, _ in re.findall(r"([^\s:]+\.(?:h|c)):(\d+):(\d+):", text):
      locs.add(((Path(f) if Path(f).is_absolute() else (ROOT / f)).resolve(), int(ln)))
  return ids, locs


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--mutator", default="all", choices=["all", *MUTATORS])
  ap.add_argument("-j", type=int, default=max((os.cpu_count() or 1) - 1, 1))
  ap.add_argument("--max-mutants", type=int, default=0)
  ap.add_argument("--list-only", action="store_true")
  args = ap.parse_args()

  clang = next((n for n in ("clang-17", "clang") if shutil.which(n)), None)
  if not clang:
    raise RuntimeError("clang not found")

  t_start = time.perf_counter()
  with tempfile.TemporaryDirectory(prefix="mutation-") as tmp:
    pp_file = Path(tmp) / "safety.i"
    print("Preprocessing...", flush=True)
    preprocess(clang, ROOT / SAFETY_C, pp_file)

    label = args.mutator if args.mutator != "all" else "all mutators"
    print(f"Discovering mutation sites for: {label}", flush=True)
    sites, counts = discover_sites(clang, pp_file, args.mutator)
    if not sites:
      print("No mutation candidates found.", flush=True)
      return 2
    if args.max_mutants > 0:
      sites = sites[:args.max_mutants]

    summary = ", ".join(f"{m} ({counts[m]})" for m in MUTATORS if counts.get(m, 0))
    print(f"Found {len(sites)} candidates: {summary}", flush=True)
    if args.list_only:
      for s in sites:
        print(f"  #{s.id:03d} {_site_loc(s)} [{s.mutator}] {s.original}->{s.replacement}")
      return 0

    # Compile with iterative error pruning
    discovered = len(sites)
    lib = Path(tmp) / "libsafety_mutation.so"
    compile_sites, pruned = sites, 0
    print(f"Compiling {len(sites)} mutants into single library...", flush=True)
    while True:
      try:
        build_sec = compile_library(clang, pp_file, compile_sites, lib)
        break
      except RuntimeError as exc:
        err = str(exc)
        before = len(compile_sites)
        bad_ids, bad_locs = _parse_errors(err)
        compile_sites = [s for s in compile_sites if s.id not in bad_ids and
                         ((s.origin_file or s.source), (s.origin_line or s.line)) not in bad_locs]
        if len(compile_sites) >= before or not compile_sites:
          print(f"Build failed:\n{err}", flush=True)
          return 2
        pruned += before - len(compile_sites)

    if pruned:
      sites = [replace(s, id=i) for i, s in enumerate(compile_sites)]
      print(f"Pruned {pruned} build-incompatible mutants", flush=True)
    else:
      sites = compile_sites

    # Baseline smoke test
    smoke = [_test_module(TESTS_DIR / n) for n in SMOKE_TESTS]
    killed, killer, _ = run_tests(smoke, lib, -1)
    if killed:
      print(f"Baseline smoke failed: {killer}", flush=True)
      return 2

    # Run all mutants in parallel
    print(f"Running {len(sites)} mutants with {args.j} workers", flush=True)
    results: list[Result] = []
    k = s = i = 0
    is_tty = sys.stdout.isatty()
    def show(text: str, final: bool = False) -> None:
      print(("\r" + text if is_tty else text), end="\n" if (final or not is_tty) else "", flush=True)

    with ProcessPoolExecutor(max_workers=args.j, max_tasks_per_child=1) as pool:
      futures: dict[Future[Result], Site] = {pool.submit(eval_mutant, st, lib): st for st in sites}
      show(progress(0, len(sites), 0, 0, 0, 0))
      for fut in as_completed(futures):
        try:
          res = fut.result()
        except Exception:
          res = Result(futures[fut], "killed", "tests", "worker-crash", 0.0)
        results.append(res)
        if res.outcome == "killed": k += 1
        elif res.outcome == "survived": s += 1
        else: i += 1
        elapsed = time.perf_counter() - t_start
        show(progress(len(results), len(sites), k, s, i, elapsed), final=(len(results) == len(sites)))

    # Report survivors
    survivors = sorted((r for r in results if r.outcome == "survived"), key=lambda r: r.site.id)
    if survivors:
      print("\nSurviving mutants", flush=True)
      for r in survivors:
        print(f"- #{r.site.id} {_site_loc(r.site)} [{r.site.mutator}] {r.site.original}->{r.site.replacement}", flush=True)
        print(snippet(r.site), flush=True)

    infra_results = sorted((r for r in results if r.outcome == "infra_error"), key=lambda r: r.site.id)
    if infra_results:
      print("\nInfra errors", flush=True)
      for r in infra_results:
        detail = r.details.splitlines()[0] if r.details else "unknown"
        print(f"- #{r.site.id} {_site_loc(r.site)} ({r.stage}): {detail}", flush=True)

    elapsed = time.perf_counter() - t_start
    test_sum = sum(r.test_sec for r in results)
    print(f"""
Mutation summary
  mutator: {label}
  discovered: {discovered}, pruned: {pruned}, total: {len(sites)}
  killed: {k}, survived: {s}, infra_error: {i}
  build: {build_sec:.2f}s, test_time: {test_sum:.2f}s, elapsed: {elapsed:.2f}s
  mutants_per_second: {len(sites) / max(elapsed, 0.01):.2f}""", flush=True)

    if i:
      return 2
    if s:
      return 1
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
