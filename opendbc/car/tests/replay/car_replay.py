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


def get_changed_platforms(cwd, database):
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
  return [f"    {d[1]}: {d[2]} → {d[3]}" for d in diffs]


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
  platforms = [platform] if platform and platform in interfaces else get_changed_platforms(cwd, database)

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
