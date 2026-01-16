#!/usr/bin/env python3
import os
os.environ['LOGPRINT'] = 'CRITICAL'

import argparse
import pickle
import re
import subprocess
import sys
import tempfile
import zstandard as zstd
from urllib.request import urlopen
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from comma_car_segments import get_comma_car_segments_database, get_url

from opendbc.car.logreader import LogReader, decompress_stream


TOLERANCE = 1e-4
DIFF_BUCKET = "car_diff"
IGNORE_FIELDS = ["cumLagMs", "canErrorCounter"]


def dict_diff(d1, d2, path="", ignore=None, tolerance=0):
  ignore = ignore or []
  diffs = []
  for key in d1.keys() | d2.keys():
    if key in ignore:
      continue
    full_path = f"{path}.{key}" if path else key
    v1, v2 = d1.get(key), d2.get(key)
    if isinstance(v1, dict) and isinstance(v2, dict):
      diffs.extend(dict_diff(v1, v2, full_path, ignore, tolerance))
    elif isinstance(v1, (int, float)) and isinstance(v2, (int, float)):
      if abs(v1 - v2) > tolerance:
        diffs.append(("change", full_path, (v1, v2)))
    elif v1 != v2:
      diffs.append(("change", full_path, (v1, v2)))
  return diffs


def load_can_messages(seg):
  parts = seg.split("/")
  url = get_url(f"{parts[0]}/{parts[1]}", parts[2])
  lr = LogReader(url, only_union_types=True)
  return list(lr.filter('can'))


def replay_segment(platform, can_msgs):
  from opendbc.car import gen_empty_fingerprint, structs
  from opendbc.car.can_definitions import CanData
  from opendbc.car.car_helpers import FRAME_FINGERPRINT, interfaces

  fingerprint = gen_empty_fingerprint()
  for can in can_msgs[:FRAME_FINGERPRINT]:
    for m in can:
      if m.src < 64:
        fingerprint[m.src][m.address] = len(m.dat)

  CarInterface = interfaces[platform]
  CP = CarInterface.get_params(platform, fingerprint, [], False, False, False)
  CI = CarInterface(CP)
  CC = structs.CarControl().as_reader()

  states, timestamps = [], []
  for i, can in enumerate(can_msgs):
    t = int(0.01 * i * 1e9)
    frames = [CanData(m.address, m.dat, m.src) for m in can]
    states.append(CI.update([(t, frames)]))
    CI.apply(CC, t)
    timestamps.append(t)
  return states, timestamps


def process_segment(args):
  platform, seg, ref_path, update = args
  try:
    can_msgs = load_can_messages(seg)
    states, timestamps = replay_segment(platform, can_msgs)
    ref_file = Path(ref_path) / f"{platform}_{seg.replace('/', '_')}.zst"

    if update:
      data = list(zip(timestamps, states, strict=True))
      ref_file.write_bytes(zstd.compress(pickle.dumps(data), 10))
      return (platform, seg, [], None)

    if not ref_file.exists():
      return (platform, seg, [], "no ref")

    ref = pickle.loads(decompress_stream(ref_file.read_bytes()))
    diffs = []
    for i, ((ts, ref_state), state) in enumerate(zip(ref, states, strict=True)):
      for diff in dict_diff(ref_state.to_dict(), state.to_dict(), ignore=IGNORE_FIELDS, tolerance=TOLERANCE):
        diffs.append((diff[1], i, diff[2], ts))
    return (platform, seg, diffs, None)
  except Exception as e:
    return (platform, seg, [], str(e))


def get_changed_platforms(cwd, database, interfaces):
  git_ref = os.environ.get("GIT_REF", "origin/master")
  changed = subprocess.check_output(["git", "diff", "--name-only", f"{git_ref}...HEAD"], cwd=cwd, encoding='utf8').strip()
  brands = set()
  patterns = [r"opendbc/car/(\w+)/", r"opendbc/dbc/(\w+?)_", r"opendbc/dbc/generator/(\w+)", r"opendbc/safety/modes/(\w+?)[_.]"]
  for line in changed.splitlines():
    for pattern in patterns:
      m = re.search(pattern, line)
      if m:
        brands.add(m.group(1).lower())
  return [p for p in interfaces if any(b in p.lower() for b in brands) and p in database]


def download_refs(ref_path, platforms, segments):
  base_url = f"https://raw.githubusercontent.com/commaai/ci-artifacts/refs/heads/{DIFF_BUCKET}"
  for platform in platforms:
    for seg in segments.get(platform, []):
      filename = f"{platform}_{seg.replace('/', '_')}.zst"
      try:
        with urlopen(f"{base_url}/{filename}") as resp:
          (Path(ref_path) / filename).write_bytes(resp.read())
      except Exception:
        pass


def run_replay(platforms, segments, ref_path, update, workers=4):
  work = [(platform, seg, ref_path, update)
          for platform in platforms for seg in segments.get(platform, [])]
  with ProcessPoolExecutor(max_workers=workers) as pool:
    return list(pool.map(process_segment, work))


def find_edges(vals, init):
  rises = []
  falls = []
  prev = init
  for i, val in enumerate(vals):
    if val and not prev:
      rises.append(i)
    if not val and prev:
      falls.append(i)
    prev = val
  return rises, falls


def render_waveform(label, vals, init):
  wave = {(False, False): "_", (True, True): "‾", (False, True): "/", (True, False): "\\"}
  line = f"  {label}:".ljust(12)
  prev = init
  for val in vals:
    line += wave[(prev, val)]
    prev = val
  if len(line) > 80:
    line = line[:80] + "..."
  return line


def format_timing(edge_type, master_edges, pr_edges, ms_per_frame):
  if not master_edges or not pr_edges:
    return None
  delta = pr_edges[0] - master_edges[0]
  if delta == 0:
    return None
  direction = "lags" if delta > 0 else "leads"
  ms = int(abs(delta) * ms_per_frame)
  return " " * 12 + f"{edge_type}: PR {direction} by {abs(delta)} frames ({ms}ms)"


def group_frames(diffs, max_gap=15):
  groups = []
  current = [diffs[0]]
  for diff in diffs[1:]:
    _, frame, _, _ = diff
    _, prev_frame, _, _ = current[-1]
    if frame <= prev_frame + max_gap:
      current.append(diff)
    else:
      groups.append(current)
      current = [diff]
  groups.append(current)
  return groups


def format_diff(diffs):
  if not diffs:
    return []

  frame_pad = 5

  old, new = diffs[0][2]
  if not (isinstance(old, bool) and isinstance(new, bool)):
    lines = [f"    frame {diff[1]}: {diff[2][0]} -> {diff[2][1]}" for diff in diffs[:10]]
    if len(diffs) > 10:
      lines.append(f"    (... {len(diffs) - 10} more)")
    return lines

  ranges = group_frames(diffs)

  # ms per frame from timestamps, fallback to 10ms on single diff
  frame_diff = diffs[-1][1] - diffs[0][1]
  frame_ms = (diffs[-1][3] - diffs[0][3]) / 1e6 / frame_diff if frame_diff else 10

  lines = []
  for range_diffs in ranges:
    start, end = max(0, range_diffs[0][1] - frame_pad), range_diffs[-1][1] + frame_pad + 1
    diff_map = {diff[1]: diff for diff in range_diffs}

    last = range_diffs[-1]
    converge_frame = last[1] + 1
    converge_val   = last[2][0]
    m_st = pr_st = not converge_val

    m_vals, pr_vals = [], []
    for frame in range(start, end):
      if frame in diff_map:
        m_st, pr_st = diff_map[frame][2]
      elif frame >= converge_frame:
        m_st = pr_st = converge_val
      m_vals.append(m_st)
      pr_vals.append(pr_st)

    lines.append(f"\n  frames {start}-{end - 1}")
    init_val = not converge_val
    lines.append(render_waveform("master", m_vals, init_val))
    lines.append(render_waveform("PR", pr_vals, init_val))

    m_rises, m_falls = find_edges(m_vals, init_val)
    pr_rises, pr_falls = find_edges(pr_vals, init_val)

    for edge_type, master_edges, pr_edges in [("rise", m_rises, pr_rises), ("fall", m_falls, pr_falls)]:
      msg = format_timing(edge_type, master_edges, pr_edges, frame_ms)
      if msg:
        lines.append(msg)

  return lines


def main(platform=None, segments_per_platform=10, update_refs=False, all_platforms=False):
  from opendbc.car.car_helpers import interfaces

  cwd = Path(__file__).resolve().parents[3]
  ref_path = cwd / DIFF_BUCKET
  if not update_refs:
    ref_path = Path(tempfile.mkdtemp())
  ref_path.mkdir(exist_ok=True)
  database = get_comma_car_segments_database()

  if all_platforms:
    print("Running all platforms...")
    platforms = [p for p in interfaces if p in database]
  elif platform and platform in interfaces:
    platforms = [platform]
  else:
    platforms = get_changed_platforms(cwd, database, interfaces)

  if not platforms:
    print("No car changes detected", file=sys.stderr)
    return 0

  segments = {p: database.get(p, [])[:segments_per_platform] for p in platforms}
  n_segments = sum(len(s) for s in segments.values())
  print(f"{'Generating' if update_refs else 'Testing'} {n_segments} segments for: {', '.join(platforms)}")

  if update_refs:
    results = run_replay(platforms, segments, ref_path, update=True)
    errors = [e for _, _, _, e in results if e]
    assert len(errors) == 0, f"Segment failures: {errors}"
    print(f"Generated {n_segments} refs to {ref_path}")
    return 0

  download_refs(ref_path, platforms, segments)
  results = run_replay(platforms, segments, ref_path, update=False)

  with_diffs = [(p, s, d) for p, s, d, e in results if d]
  errors = [(p, s, e) for p, s, d, e in results if e]
  n_passed = len(results) - len(with_diffs) - len(errors)

  print(f"\nResults: {n_passed} passed, {len(with_diffs)} with diffs, {len(errors)} errors")

  for plat, seg, err in errors:
    print(f"\nERROR {plat} - {seg}: {err}")

  if with_diffs:
    print("```")
    for plat, seg, diffs in with_diffs:
      print(f"\n{plat} - {seg}")
      by_field = defaultdict(list)
      for d in diffs:
        by_field[d[0]].append(d)
      for field, fd in sorted(by_field.items()):
        print(f"  {field} ({len(fd)} diffs)")
        for line in format_diff(fd):
          print(line)
    print("```")

  return 1 if errors else 0


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--platform", help="diff single platform")
  parser.add_argument("--segments-per-platform", type=int, default=10, help="number of segments to diff per platform")
  parser.add_argument("--update-refs", action="store_true", help="update refs based on current commit")
  parser.add_argument("--all", action="store_true", help="run diff on all platforms")
  args = parser.parse_args()
  sys.exit(main(args.platform, args.segments_per_platform, args.update_refs, args.all))
