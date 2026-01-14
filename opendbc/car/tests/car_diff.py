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


def bucket_is_empty():
  from openpilot.tools.lib.github_utils import GithubUtils
  return requests.head(GithubUtils(None, None, "elkoled").get_bucket_link(DIFF_BUCKET)).status_code != 200


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
  from openpilot.tools.lib.github_utils import GithubUtils
  base_url = GithubUtils(None, None, "elkoled").get_bucket_link(DIFF_BUCKET)
  for platform in platforms:
    for seg in segments.get(platform, []):
      filename = f"{platform}_{seg.replace('/', '_')}.zst"
      resp = requests.get(f"{base_url}/{filename}")
      if resp.status_code == 200:
        (Path(ref_path) / filename).write_bytes(resp.content)


def upload_refs(ref_path, platforms, segments):
  from openpilot.tools.lib.github_utils import GithubUtils
  gh = GithubUtils(None, os.environ.get("GITHUB_TOKEN"), "elkoled")
  files = []
  for platform in platforms:
    for seg in segments.get(platform, []):
      filename = f"{platform}_{seg.replace('/', '_')}.zst"
      local_path = Path(ref_path) / filename
      if local_path.exists():
        files.append((filename, str(local_path)))
  gh.upload_files(DIFF_BUCKET, files)


def run_replay(platforms, segments, ref_path, update, workers=8):
  work = [(platform, seg, ref_path, update)
          for platform in platforms for seg in segments.get(platform, [])]
  with ProcessPoolExecutor(max_workers=workers) as pool:
    return list(pool.map(process_segment, work))


def main(platform=None, segments_per_platform=10, update_refs=False):
  from opendbc.car.car_helpers import interfaces
  from openpilot.tools.lib.comma_car_segments import get_comma_car_segments_database

  cwd = Path(__file__).resolve().parents[3]
  ref_path = tempfile.mkdtemp(prefix="car_ref_")
  database = get_comma_car_segments_database()

  if update_refs and bucket_is_empty():
    print("Bootstrapping all platforms...")
    platforms = [p for p in interfaces if p in database]
  elif platform and platform in interfaces:
    platforms = [platform]
  else:
    # auto detect platform changes by default
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
    for d in diffs:
      by_field[d[0]].append(d)
    for field, fd in sorted(by_field.items()):
      print(f"  {field} ({len(fd)} diffs)")
      for d in fd[:10]:
        print(f"    {d[1]}: {d[2][0]} -> {d[2][1]}")
      if len(fd) > 10:
        print(f"    ... ({len(fd) - 10} more)")

  return 0


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--platform", help="diff single platform")
  parser.add_argument("--segments-per-platform", type=int, default=10, help="number of segments to diff per platform")
  parser.add_argument("--update-refs", action="store_true", help="update refs based on current commit")
  args = parser.parse_args()
  sys.exit(main(args.platform, args.segments_per_platform, args.update_refs))
