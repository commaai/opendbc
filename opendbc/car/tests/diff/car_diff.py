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

import matplotlib.pyplot as plt

os.environ['LOGPRINT'] = 'ERROR'

DIFF_BUCKET = "car_diff"


def bucket_is_empty():
  from openpilot.tools.lib.github_utils import GithubUtils
  return requests.head(GithubUtils(None, None).get_bucket_link(DIFF_BUCKET)).status_code != 200


def get_changed_platforms(cwd, database, interfaces):
  from openpilot.common.utils import run_cmd
  git_ref = os.environ.get("GIT_REF", "origin/master")
  changed = run_cmd(["git", "diff", "--name-only", f"{git_ref}...HEAD"], cwd=cwd)
  brands = set()
  for line in changed.splitlines():
    if m := re.search(r"opendbc/car/(\w+)/", line):
      brands.add(m.group(1))
    if m := re.search(r"opendbc/dbc/(\w+)_", line):
      brands.add(m.group(1).lower())
    if m := re.search(r"opendbc/safety/modes/(\w+)[_.]", line):
      brands.add(m.group(1).lower())
  return [p for p in interfaces if any(b.upper() in p for b in brands) and p in database]


def download_refs(ref_path, platforms, segments):
  from openpilot.tools.lib.github_utils import GithubUtils
  gh = GithubUtils(None, None)
  base_url = gh.get_bucket_link(DIFF_BUCKET)
  for platform in platforms:
    for seg in segments.get(platform, []):
      filename = f"{platform}_{seg.replace('/', '_')}.zst"
      resp = requests.get(f"{base_url}/{filename}")
      if resp.status_code == 200:
        (Path(ref_path) / filename).write_bytes(resp.content)


def upload_refs(ref_path, platforms, segments):
  from openpilot.tools.lib.github_utils import GithubUtils
  gh = GithubUtils(None, os.environ.get("GITHUB_TOKEN"))
  files = []
  for platform in platforms:
    for seg in segments.get(platform, []):
      filename = f"{platform}_{seg.replace('/', '_')}.zst"
      local_path = Path(ref_path) / filename
      if local_path.exists():
        files.append((filename, str(local_path)))
  gh.upload_files(DIFF_BUCKET, files)


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
      for i, v in enumerate(vals[:100]):
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


def plot_diff(field, diffs, output_dir):
  if not diffs:
    return None

  diff_map = {d[1]: (d[2], d[3]) for d in diffs}
  frames = sorted(diff_map.keys())
  is_bool = all(isinstance(d[2], bool) and isinstance(d[3], bool) for d in diffs)

  # group into edge regions (gaps > 15 frames = new region)
  regions, cur = [], [frames[0]]
  for f in frames[1:]:
    if f - cur[-1] > 15:
      regions.append(cur)
      cur = [f]
    else:
      cur.append(f)
  regions.append(cur)

  if is_bool:
    n_regions = min(len(regions), 6)
    fig, axes = plt.subplots(n_regions, 1, figsize=(8, 1.5 * n_regions), squeeze=False)

    for idx in range(n_regions):
      region_frames = regions[idx]
      ax = axes[idx, 0]
      f_min, f_max = min(region_frames) - 5, max(region_frames) + 5
      x_vals = list(range(f_min, f_max + 1))

      first_m, first_p = diff_map[region_frames[0]]
      if first_m and not first_p:
        m_state, p_state = False, False
      elif not first_m and first_p:
        m_state, p_state = True, True
      else:
        m_state, p_state = False, False

      last_frame = region_frames[-1]
      converge_val = diff_map[last_frame][0]

      m_vals, p_vals = [], []
      for x in x_vals:
        if x in diff_map:
          m_state, p_state = diff_map[x]
        elif x > last_frame:
          m_state = p_state = converge_val
        m_vals.append(int(m_state))
        p_vals.append(int(p_state))

      ax.step(x_vals, m_vals, where='post', label='master', color='#1f77b4', linewidth=2)
      ax.step(x_vals, p_vals, where='post', label='PR', color='#ff7f0e', linewidth=2)

      m_rises = [i for i, v in enumerate(m_vals) if v and (i == 0 or not m_vals[i - 1])]
      p_rises = [i for i, v in enumerate(p_vals) if v and (i == 0 or not p_vals[i - 1])]
      m_falls = [i for i, v in enumerate(m_vals) if not v and i > 0 and m_vals[i - 1]]
      p_falls = [i for i, v in enumerate(p_vals) if not v and i > 0 and p_vals[i - 1]]

      ts_list = [(d[1], d[4]) for d in diffs if d[1] in region_frames and len(d) > 4]
      frame_ms = 10
      if len(ts_list) >= 2:
        ts_list.sort()
        frame_ms = (ts_list[-1][1] - ts_list[0][1]) / 1e6 / (ts_list[-1][0] - ts_list[0][0])

      annotation = ""
      if m_rises and p_rises and (delta := p_rises[0] - m_rises[0]):
        annotation = f"PR {'lags' if delta > 0 else 'leads'} by {int(abs(delta) * frame_ms)} ms"
      if m_falls and p_falls and (delta := p_falls[0] - m_falls[0]):
        annotation = f"PR {'lags' if delta > 0 else 'leads'} by {int(abs(delta) * frame_ms)} ms"

      ax.set_yticks([0, 1])
      ax.set_yticklabels(['F', 'T'])
      ax.set_ylim(-0.1, 1.1)
      ax.set_xlim(f_min, f_max)
      ax.tick_params(axis='x', labelsize=8)
      if annotation:
        ax.text(0.02, 0.5, annotation, transform=ax.transAxes, fontsize=9, ha='left', va='center')
      if idx == 0:
        ax.legend(loc='upper right', fontsize=8)
    axes[0, 0].set_title(field, fontsize=10)

  else:
    fig, ax = plt.subplots(figsize=(8, 2.5))
    m_vals = [diff_map[f][0] for f in frames]
    p_vals = [diff_map[f][1] for f in frames]
    pad = max(10, (max(frames) - min(frames)) // 10)
    ax.plot(frames, m_vals, '-', label='master', color='#1f77b4', linewidth=1.5)
    ax.plot(frames, p_vals, '--', label='PR', color='#ff7f0e', linewidth=1.5)
    ax.set_xlim(min(frames) - pad, max(frames) + pad)
    ax.set_title(field, fontsize=10)
    ax.legend(loc='upper right', fontsize=8)
    ax.tick_params(axis='x', labelsize=8)

  plt.tight_layout()
  plot_path = output_dir / f'{field.replace(".", "_")}.png'
  plt.savefig(plot_path, dpi=120, facecolor='white')
  plt.close()
  return plot_path


def run_replay(platforms, segments, ref_path, update, workers=8):
  from opendbc.car.tests.diff.replay import process_segment
  work = [(platform, seg, ref_path, update)
          for platform in platforms for seg in segments.get(platform, [])]
  with ProcessPoolExecutor(max_workers=workers) as pool:
    return list(pool.map(process_segment, work))


def main(platform=None, segments_per_platform=10, update_refs=False, plot=False):
  from opendbc.car.car_helpers import interfaces
  from openpilot.tools.lib.comma_car_segments import get_comma_car_segments_database

  cwd = Path(__file__).resolve().parents[4]
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

  plot_dir = Path(tempfile.mkdtemp(prefix="car_replay_plots_")) if plot else None
  upload = plot and os.environ.get("AZURE_TOKEN")
  if upload:
    from azure.storage.blob import BlobClient, ContentSettings
    from openpilot.tools.lib.azure_container import get_azure_credential
    account_url = "https://elkoled.blob.core.windows.net"
    container_name = "openpilotci"
  image_urls = []

  if with_diffs:
    print("```")  # open code block for ASCII output

  for plat, seg, diffs in with_diffs:
    print(f"\n{plat} - {seg}")
    by_field = defaultdict(list)
    for d in diffs:
      by_field[d[0]].append(d)

    if plot_dir:
      seg_dir = plot_dir / f"{plat}_{seg.replace('/', '_')}"
      seg_dir.mkdir(exist_ok=True)

    for field, fd in sorted(by_field.items()):
      print(f"  {field} (frame: master → PR)")
      for line in format_diff(fd):
        print(line)
      if plot_dir:
        plot_path = plot_diff(field, fd, seg_dir)
        if plot_path and upload:
          blob_name = f"car_replay_plots/{seg_dir.name}/{plot_path.name}"
          blob = BlobClient(account_url, container_name, blob_name, credential=get_azure_credential())
          with open(plot_path, "rb") as f:
            blob.upload_blob(f, overwrite=True, content_settings=ContentSettings(content_type="image/png"))
          image_urls.append((plat, seg, field, f"{account_url}/{container_name}/{blob_name}"))

  if with_diffs:
    print("```")  # close code block

  # print images outside code block so markdown renders them
  if image_urls:
    print("\n**Plots:**")
    for plat, seg, field, url in image_urls:
      print(f"\n{plat} - {field}")
      print(f"![{field}]({url})")

  return 0


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--platform", help="run diff on a single platform only")
  parser.add_argument("--segments-per-platform", type=int, default=10, help="number of segments to test per platform")
  parser.add_argument("--update-refs", action="store_true", help="update refs to the current commit")
  args = parser.parse_args()
  sys.exit(main(args.platform, args.segments_per_platform, args.update_refs, args.plot))
