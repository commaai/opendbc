#!/usr/bin/env python3
import os
os.environ['LOGPRINT'] = 'CRITICAL'

import argparse
import pickle
import re
import requests
import sys
import tempfile
import zstandard as zstd
import dictdiffer
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

DIFF_BUCKET = "car_diff"
TOLERANCE = 1e-2
IGNORE_FIELDS = ["cumLagMs", "canErrorCounter"]


def load_can_messages(seg):
  from opendbc.car.can_definitions import CanData
  from openpilot.selfdrive.pandad import can_capnp_to_list
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.tools.lib.comma_car_segments import get_url

  parts = seg.split("/")
  url = get_url(f"{parts[0]}/{parts[1]}", parts[2])

  can_msgs = []
  for msg in LogReader(url):
    if msg.which() == "can":
      can = can_capnp_to_list((msg.as_builder().to_bytes(),))[0]
      can_msgs.append((can[0], [CanData(*c) for c in can[1]]))
  return can_msgs


def replay_segment(platform, can_msgs):
  from opendbc.car import gen_empty_fingerprint, structs
  from opendbc.car.car_helpers import FRAME_FINGERPRINT, interfaces

  fingerprint = gen_empty_fingerprint()
  for _, frames in can_msgs[:FRAME_FINGERPRINT]:
    for msg in frames:
      if msg.src < 64:
        fingerprint[msg.src][msg.address] = len(msg.dat)

  CarInterface = interfaces[platform]
  car_interface = CarInterface(CarInterface.get_params(platform, fingerprint, [], False, False, False))
  car_control = structs.CarControl().as_reader()

  states, timestamps = [], []
  for ts, frames in can_msgs:
    states.append(car_interface.update([(ts, frames)]))
    car_interface.apply(car_control, ts)
    timestamps.append(ts)
  return states, timestamps


def process_segment(args):
  platform, seg, ref_path, update = args
  try:
    can_msgs = load_can_messages(seg)
    states, timestamps = replay_segment(platform, can_msgs)
    ref_file = Path(ref_path) / f"{platform}_{seg.replace('/', '_')}.zst"

    if update:
      data = list(zip(timestamps, states, strict=True))
      ref_file.write_bytes(zstd.compress(pickle.dumps(data)))
      return (platform, seg, [], None)

    if not ref_file.exists():
      return (platform, seg, [], "no ref")

    ref = pickle.loads(zstd.decompress(ref_file.read_bytes()))
    diffs = []
    for i, ((ts, ref_state), state) in enumerate(zip(ref, states, strict=True)):
      for diff in dictdiffer.diff(ref_state.to_dict(), state.to_dict(), ignore=IGNORE_FIELDS, tolerance=TOLERANCE):
        if diff[0] == "change":  # ignore add/remove from schema changes
          diffs.append((str(diff[1]), i, diff[2], ts))
    return (platform, seg, diffs, None)
  except Exception as e:
    return (platform, seg, [], str(e))


def get_changed_platforms(cwd, database, interfaces):
  from openpilot.common.utils import run_cmd
  git_ref = os.environ.get("GIT_REF", "origin/master")
  changed = run_cmd(["git", "diff", "--name-only", f"{git_ref}...HEAD"], cwd=cwd)
  brands = set()
  patterns = [r"opendbc/car/(\w+)/", r"opendbc/dbc/(\w+)_", r"opendbc/safety/modes/(\w+)[_.]"]
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
      resp = requests.get(f"{base_url}/{filename}")
      if resp.status_code == 200:
        (Path(ref_path) / filename).write_bytes(resp.content)




def run_replay(platforms, segments, ref_path, update, workers=8):
  work = [(platform, seg, ref_path, update)
          for platform in platforms for seg in segments.get(platform, [])]
  with ProcessPoolExecutor(max_workers=workers) as pool:
    return list(pool.map(process_segment, work))


def format_diff(diffs):
  if not diffs:
    return []
  if not all(isinstance(d[2], bool) and isinstance(d[3], bool) for d in diffs):
    return [f"    frame {d[1]}: {d[2]} -> {d[3]}" for d in diffs[:10]]

  lines = []
  ranges, cur = [], [diffs[0]]
  for d in diffs[1:]:
    if d[1] <= cur[-1][1] + 15:
      cur.append(d)
    else:
      ranges.append(cur)
      cur = [d]
  ranges.append(cur)

  for rdiffs in ranges:
    t0, t1 = max(0, rdiffs[0][1] - 5), rdiffs[-1][1] + 6
    diff_map = {d[1]: d for d in rdiffs}

    b_vals, m_vals, ts_map = [], [], {}
    last = rdiffs[-1]
    converge_frame = last[1] + 1
    converge_val = last[2]
    b_st = m_st = not converge_val  # before divergence, both had opposite of converge value

    for f in range(t0, t1):
      if f in diff_map:
        b_st, m_st = diff_map[f][2], diff_map[f][3]
        if len(diff_map[f]) > 4:
          ts_map[f] = diff_map[f][4]
      elif f >= converge_frame:
        b_st = m_st = converge_val
      b_vals.append(b_st)
      m_vals.append(m_st)

    ts_start = ts_map.get(t0, rdiffs[0][4] if len(rdiffs[0]) > 4 else 0)
    ts_end = ts_map.get(t1 - 1, rdiffs[-1][4] if len(rdiffs[-1]) > 4 else 0)
    t0_sec = ts_start / 1e9
    t1_sec = ts_end / 1e9

    # ms per frame from timestamps
    if len(ts_map) >= 2:
      ts_vals = sorted(ts_map.items())
      frame_ms = (ts_vals[-1][1] - ts_vals[0][1]) / 1e6 / (ts_vals[-1][0] - ts_vals[0][0])
    else:
      frame_ms = 10

    lines.append(f"\n  frames {t0}-{t1-1}")
    pad = 12
    max_width = 60
    init_val = not converge_val
    for label, vals, init in [("master", b_vals, init_val), ("PR", m_vals, init_val)]:
      line = f"  {label}:".ljust(pad)
      for i, v in enumerate(vals):
        pv = vals[i - 1] if i > 0 else init
        if v and not pv:
          line += "/"
        elif not v and pv:
          line += "\\"
        elif v:
          line += "‾"
        else:
          line += "_"
      if len(line) > pad + max_width:
        line = line[:pad + max_width] + "..."
      lines.append(line)

    b_rises = [i for i, v in enumerate(b_vals) if v and (i == 0 or not b_vals[i - 1])]
    m_rises = [i for i, v in enumerate(m_vals) if v and (i == 0 or not m_vals[i - 1])]
    b_falls = [i for i, v in enumerate(b_vals) if not v and i > 0 and b_vals[i - 1]]
    m_falls = [i for i, v in enumerate(m_vals) if not v and i > 0 and m_vals[i - 1]]

    if b_rises and m_rises:
      delta = m_rises[0] - b_rises[0]
      if delta:
        ms = int(abs(delta) * frame_ms)
        direction = "lags" if delta > 0 else "leads"
        lines.append(" " * pad + f"rise: PR {direction} by {abs(delta)} frames ({ms}ms)")
    if b_falls and m_falls:
      delta = m_falls[0] - b_falls[0]
      if delta:
        ms = int(abs(delta) * frame_ms)
        direction = "lags" if delta > 0 else "leads"
        lines.append(" " * pad + f"fall: PR {direction} by {abs(delta)} frames ({ms}ms)")

  return lines


def main(platform=None, segments_per_platform=10, update_refs=False, all_platforms=False):
  from opendbc.car.car_helpers import interfaces
  from openpilot.tools.lib.comma_car_segments import get_comma_car_segments_database

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
