#!/usr/bin/env python3
import argparse
import csv
import html
import math
from pathlib import Path


CHART_W = 1100
CHART_H = 320
PAD_L = 72
PAD_R = 20
PAD_T = 24
PAD_B = 40


def parse_float(value, default=math.nan):
  try:
    return float(value)
  except (TypeError, ValueError):
    return default


def load_rows(path):
  with path.open(newline="", encoding="utf-8") as f:
    return list(csv.DictReader(f))


def x_values(rows):
  ts = [parse_float(row.get("t")) for row in rows]
  if all(math.isfinite(t) for t in ts):
    t0 = ts[0]
    return [t - t0 for t in ts]

  frames = [parse_float(row.get("frame"), i) for i, row in enumerate(rows)]
  return [frame * 0.01 for frame in frames]


def series(rows, key):
  return [parse_float(row.get(key)) for row in rows]


def finite_bounds(values):
  finite = [v for v in values if math.isfinite(v)]
  if not finite:
    return -1., 1.

  low = min(finite)
  high = max(finite)
  if low == high:
    span = max(abs(low) * 0.1, 1.)
    return low - span, high + span

  span = high - low
  return low - span * 0.06, high + span * 0.06


def fmt(v):
  if abs(v) >= 100:
    return f"{v:.0f}"
  if abs(v) >= 10:
    return f"{v:.1f}"
  return f"{v:.2f}"


def project(x, y, xb, yb):
  x0, x1 = xb
  y0, y1 = yb
  px = PAD_L + (x - x0) / (x1 - x0) * (CHART_W - PAD_L - PAD_R)
  py = PAD_T + (1. - (y - y0) / (y1 - y0)) * (CHART_H - PAD_T - PAD_B)
  return px, py


def line_points(xs, ys, xb, yb):
  points = []
  for x, y in zip(xs, ys, strict=True):
    if not math.isfinite(x) or not math.isfinite(y):
      continue
    px, py = project(x, y, xb, yb)
    points.append(f"{px:.1f},{py:.1f}")
  return " ".join(points)


def axis_grid(xb, yb):
  pieces = []
  plot_w = CHART_W - PAD_L - PAD_R
  plot_h = CHART_H - PAD_T - PAD_B

  for i in range(6):
    frac = i / 5.
    x = PAD_L + frac * plot_w
    value = xb[0] + frac * (xb[1] - xb[0])
    pieces.append(f'<line class="grid" x1="{x:.1f}" y1="{PAD_T}" x2="{x:.1f}" y2="{CHART_H - PAD_B}"/>')
    pieces.append(f'<text class="tick x" x="{x:.1f}" y="{CHART_H - 14}">{html.escape(fmt(value))}</text>')

  for i in range(5):
    frac = i / 4.
    y = PAD_T + frac * plot_h
    value = yb[1] - frac * (yb[1] - yb[0])
    pieces.append(f'<line class="grid" x1="{PAD_L}" y1="{y:.1f}" x2="{CHART_W - PAD_R}" y2="{y:.1f}"/>')
    pieces.append(f'<text class="tick y" x="{PAD_L - 10}" y="{y + 4:.1f}">{html.escape(fmt(value))}</text>')

  pieces.append(f'<line class="axis" x1="{PAD_L}" y1="{PAD_T}" x2="{PAD_L}" y2="{CHART_H - PAD_B}"/>')
  pieces.append(f'<line class="axis" x1="{PAD_L}" y1="{CHART_H - PAD_B}" x2="{CHART_W - PAD_R}" y2="{CHART_H - PAD_B}"/>')
  return "\n".join(pieces)


def chart(title, rows, xs, line_defs, ylabel):
  colors = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9"]
  plotted = []
  all_y = []
  for idx, (key, label) in enumerate(line_defs):
    ys = series(rows, key)
    plotted.append((key, label, ys, colors[idx % len(colors)]))
    all_y.extend(ys)

  xb = finite_bounds(xs)
  yb = finite_bounds(all_y)

  lines = []
  for _key, label, ys, color in plotted:
    points = line_points(xs, ys, xb, yb)
    lines.append(f'<polyline class="trace" points="{points}" stroke="{color}"><title>{html.escape(label)}</title></polyline>')

  legend = []
  for _key, label, _ys, color in plotted:
    legend.append(f'<span class="legend-item"><span class="swatch" style="background:{color}"></span>{html.escape(label)}</span>')

  return f"""
  <section class="chart">
    <div class="chart-head">
      <h2>{html.escape(title)}</h2>
      <div class="legend">{''.join(legend)}</div>
    </div>
    <svg viewBox="0 0 {CHART_W} {CHART_H}" role="img" aria-label="{html.escape(title)}">
      <text class="ylabel" x="18" y="{CHART_H / 2:.1f}" transform="rotate(-90 18 {CHART_H / 2:.1f})">{html.escape(ylabel)}</text>
      {axis_grid(xb, yb)}
      {''.join(lines)}
      <text class="xlabel" x="{CHART_W / 2:.1f}" y="{CHART_H - 3}">time, s</text>
    </svg>
  </section>
  """


def build_html(csv_path, rows, xs):
  charts = [
    chart("Wheel Speed Setpoint vs Measured", rows, xs, [
      ("wheel_l_desired", "left setpoint"),
      ("wheel_l_measured", "left measured"),
      ("wheel_r_desired", "right setpoint"),
      ("wheel_r_measured", "right measured"),
    ], "m/s"),
    chart("Body Speed Loop", rows, xs, [
      ("speed_desired", "speed setpoint"),
      ("speed_measured", "speed measured"),
      ("speed_error", "speed error"),
    ], "m/s"),
    chart("Turn Loop", rows, xs, [
      ("speed_diff_desired", "turn setpoint"),
      ("speed_diff_measured", "turn measured"),
      ("turn_error", "turn error"),
    ], "m/s"),
    chart("Torque Output", rows, xs, [
      ("torque_l_raw", "left raw"),
      ("torque_l_cmd", "left CAN"),
      ("torque_r_raw", "right raw"),
      ("torque_r_cmd", "right CAN"),
    ], "torque counts"),
    chart("PID Terms", rows, xs, [
      ("speed_pid_p", "speed P"),
      ("speed_pid_i", "speed I"),
      ("turn_pid_p", "turn P"),
      ("turn_pid_i", "turn I"),
    ], "torque counts"),
  ]

  start = xs[0] if xs else 0.
  end = xs[-1] if xs else 0.
  return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Body PID Plot</title>
<style>
  :root {{
    color-scheme: light;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    background: #f6f7f8;
    color: #1f2933;
  }}
  body {{
    margin: 0;
    padding: 28px;
  }}
  main {{
    max-width: 1180px;
    margin: 0 auto;
  }}
  header {{
    margin-bottom: 20px;
  }}
  h1 {{
    margin: 0 0 8px;
    font-size: 28px;
    font-weight: 650;
  }}
  .meta {{
    color: #52616f;
    font-size: 14px;
  }}
  .chart {{
    background: #ffffff;
    border: 1px solid #d8dee4;
    border-radius: 8px;
    margin: 16px 0;
    padding: 16px;
  }}
  .chart-head {{
    display: flex;
    align-items: baseline;
    justify-content: space-between;
    gap: 16px;
    margin-bottom: 8px;
  }}
  h2 {{
    margin: 0;
    font-size: 17px;
    font-weight: 650;
  }}
  .legend {{
    display: flex;
    flex-wrap: wrap;
    justify-content: flex-end;
    gap: 8px 14px;
    font-size: 12px;
    color: #35414d;
  }}
  .legend-item {{
    white-space: nowrap;
  }}
  .swatch {{
    display: inline-block;
    width: 10px;
    height: 10px;
    margin-right: 5px;
    vertical-align: -1px;
  }}
  svg {{
    display: block;
    width: 100%;
    height: auto;
  }}
  .grid {{
    stroke: #e6e9ed;
    stroke-width: 1;
  }}
  .axis {{
    stroke: #7b8794;
    stroke-width: 1.2;
  }}
  .tick, .xlabel, .ylabel {{
    fill: #66717d;
    font-size: 12px;
  }}
  .tick.x {{
    text-anchor: middle;
  }}
  .tick.y {{
    text-anchor: end;
  }}
  .xlabel {{
    text-anchor: middle;
  }}
  .ylabel {{
    text-anchor: middle;
  }}
  .trace {{
    fill: none;
    stroke-width: 2;
    stroke-linejoin: round;
    stroke-linecap: round;
  }}
</style>
</head>
<body>
<main>
  <header>
    <h1>Body PID Plot</h1>
    <div class="meta">{html.escape(str(csv_path))} | {len(rows)} samples | {start:.2f}s to {end:.2f}s</div>
  </header>
  {''.join(charts)}
</main>
</body>
</html>
"""


def main():
  parser = argparse.ArgumentParser(description="Plot body CarController PID debug CSV as a self-contained HTML file.")
  parser.add_argument("csv", type=Path, help="CSV produced by BODY_CONTROLLER_DEBUG_CSV")
  parser.add_argument("-o", "--output", type=Path, help="Output HTML path. Defaults to CSV path with .html suffix")
  args = parser.parse_args()

  rows = load_rows(args.csv)
  if not rows:
    raise SystemExit(f"No rows found in {args.csv}")

  xs = x_values(rows)
  output = args.output if args.output is not None else args.csv.with_suffix(".html")
  output.write_text(build_html(args.csv, rows, xs), encoding="utf-8")
  print(output)


if __name__ == "__main__":
  main()
