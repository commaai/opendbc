#!/usr/bin/env python3
import argparse
import os
import re
import requests
import sys
import tempfile
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

os.environ['LOGPRINT'] = 'ERROR'

def get_changed_platforms(cwd, database, interfaces):
  from openpilot.common.utils import run_cmd
  git_ref = os.environ.get("GIT_REF", "origin/master")
  changed = run_cmd(["git", "diff", "--name-only", f"{git_ref}...HEAD"], cwd=cwd)
  brands = set()
  for line in changed.splitlines():
    if m := re.search(r"opendbc/car/(\w+)/", line):
      brands.add(m.group(1))
    if m := re.search(r"opendbc/dbc/(\w+?)_", line):
      brands.add(m.group(1).lower())
    if m := re.search(r"opendbc/safety/modes/(\w+?)[_.]", line):
      brands.add(m.group(1).lower())
  return [p for p in interfaces if any(b.upper() in p for b in brands) and p in database]


def download_refs(ref_path, platforms, segments):
  from concurrent.futures import ThreadPoolExecutor
  BASE_URL = "https://elkoled.blob.core.windows.net/openpilotci/"
  def fetch(item):
    platform, seg = item
    filename = f"{platform}_{seg.replace('/', '_')}.zst"
    resp = requests.get(f"{BASE_URL}car_replay/{filename}")
    if resp.status_code == 200:
      (Path(ref_path) / filename).write_bytes(resp.content)
  work = [(p, s) for p in platforms for s in segments.get(p, [])]
  with ThreadPoolExecutor(max_workers=8) as pool:
    list(pool.map(fetch, work))


def upload_refs(ref_path, platforms, segments):
  from openpilot.tools.lib.azure_container import AzureContainer
  container = AzureContainer("elkoled", "openpilotci")
  for platform in platforms:
    for seg in segments.get(platform, []):
      filename = f"{platform}_{seg.replace('/', '_')}.zst"
      local_path = Path(ref_path) / filename
      if local_path.exists():
        container.upload_file(str(local_path), f"car_replay/{filename}", overwrite=True)


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
    first, last = rdiffs[0], rdiffs[-1]
    if first[2] and not first[3]:
      b_st, m_st = False, False
    elif not first[2] and first[3]:
      b_st, m_st = True, True
    else:
      b_st, m_st = False, False

    converge_frame = last[1] + 1
    converge_val = last[2]

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
    init_b = not (first[2] and not first[3])
    init_m = not first[2] and first[3]
    for label, vals, init in [("master", b_vals, init_b), ("PR", m_vals, init_m)]:
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


def run_replay(platforms, segments, ref_path, update, workers=8):
  from opendbc.car.tests.replay.worker import process_segment
  work = [(platform, seg, ref_path, update)
          for platform in platforms for seg in segments.get(platform, [])]
  with ProcessPoolExecutor(max_workers=workers) as pool:
    return list(pool.map(process_segment, work))


def main(platform=None, segments_per_platform=10, update_refs=False):
  from opendbc.car.car_helpers import interfaces
  from openpilot.tools.lib.comma_car_segments import get_comma_car_segments_database

  cwd = Path(__file__).resolve().parents[4]
  ref_path = tempfile.mkdtemp(prefix="car_ref_")
  database = get_comma_car_segments_database()
  platforms = [platform] if platform and platform in interfaces else get_changed_platforms(cwd, database, interfaces)

  if not platforms:
    print("No platforms detected from changes")
    return 0

  segments = {p: database.get(p, [])[:segments_per_platform] for p in platforms}
  n_segments = sum(len(s) for s in segments.values())
  print(f"{'Generating' if update_refs else 'Testing'} {n_segments} segments for: {', '.join(platforms)}")

  if update_refs:
    results = run_replay(platforms, segments, ref_path, update=True)
    errors = [e for _, _, _, e in results if e]
    assert len(errors) == 0, f"Segment failures: {errors}"
    upload_refs(ref_path, platforms, segments)
    print(f"Uploaded {n_segments} refs")
    return 0

  download_refs(ref_path, platforms, segments)
  results = run_replay(platforms, segments, ref_path, update=False)

  with_diffs = [(p, s, d) for p, s, d, e in results if d]
  errors = [(p, s, e) for p, s, d, e in results if e]
  n_passed = len(results) - len(with_diffs) - len(errors)

  print(f"\nResults: {n_passed} passed, {len(with_diffs)} with diffs, {len(errors)} errors")

  for plat, seg, err in errors:
    print(f"\nERROR {plat} - {seg}: {err}")

  for plat, seg, diffs in with_diffs:
    print(f"\n{plat} - {seg}")
    by_field = defaultdict(list)
    for d in diffs[:100]:
      by_field[d[0]].append(d)
    for field, fd in sorted(by_field.items()):
      print(f"  {field} (frame: master → PR)")
      for line in format_diff(fd):
        print(line)
    if len(diffs) > 100:
      print(f"    ... ({len(diffs) - 100} more)")

  return 0


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--platform")
  parser.add_argument("--segments-per-platform", type=int, default=10)
  parser.add_argument("--update-refs", action="store_true")
  args = parser.parse_args()
  sys.exit(main(args.platform, args.segments_per_platform, args.update_refs))
