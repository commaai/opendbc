import atexit
import html
import json
import math
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8765
DEFAULT_MAX_ROWS = 6000


class DaemonThreadingHTTPServer(ThreadingHTTPServer):
  daemon_threads = True


def _json_value(value):
  if hasattr(value, "item"):
    value = value.item()

  if isinstance(value, bool | int | str) or value is None:
    return value

  if isinstance(value, float):
    return value if math.isfinite(value) else None

  return str(value)


def _display_url(host: str, port: int) -> str:
  if host in ("0.0.0.0", "::"):
    return f"http://<device-ip>:{port}"
  return f"http://{host}:{port}"


def dashboard_html(source: str, max_rows: int) -> str:
  return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Live Body PID</title>
<style>
  :root {{
    color-scheme: light;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    background: #f4f6f8;
    color: #1f2933;
  }}
  body {{
    margin: 0;
    padding: 18px;
  }}
  main {{
    max-width: 1260px;
    margin: 0 auto;
  }}
  header {{
    display: flex;
    align-items: flex-start;
    justify-content: space-between;
    gap: 16px;
    margin-bottom: 14px;
  }}
  h1 {{
    margin: 0 0 6px;
    font-size: 24px;
    font-weight: 650;
  }}
  .meta, .status {{
    font-size: 13px;
    color: #52616f;
  }}
  .meta {{
    overflow-wrap: anywhere;
  }}
  .toolbar {{
    display: flex;
    align-items: center;
    gap: 8px;
    flex-wrap: wrap;
    justify-content: flex-end;
  }}
  button, select {{
    height: 34px;
    border: 1px solid #b8c1cc;
    background: #fff;
    color: #26313d;
    border-radius: 6px;
    padding: 0 10px;
    font: inherit;
    font-size: 13px;
  }}
  button.active {{
    background: #26313d;
    color: #fff;
    border-color: #26313d;
  }}
  .status {{
    min-height: 20px;
    margin-bottom: 10px;
  }}
  .grid {{
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 12px;
  }}
  .panel {{
    background: #fff;
    border: 1px solid #d7dde4;
    border-radius: 8px;
    padding: 12px;
    min-width: 0;
  }}
  .wide {{
    grid-column: 1 / -1;
  }}
  .panel-head {{
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    gap: 12px;
    margin-bottom: 8px;
  }}
  h2 {{
    margin: 0;
    font-size: 15px;
    font-weight: 650;
  }}
  .legend {{
    display: flex;
    flex-wrap: wrap;
    justify-content: flex-end;
    gap: 6px 12px;
    font-size: 12px;
    color: #394653;
  }}
  .legend span {{
    white-space: nowrap;
  }}
  .swatch {{
    display: inline-block;
    width: 10px;
    height: 10px;
    margin-right: 5px;
    vertical-align: -1px;
  }}
  canvas {{
    display: block;
    width: 100%;
    height: 230px;
  }}
  @media (max-width: 900px) {{
    body {{
      padding: 10px;
    }}
    header {{
      display: block;
    }}
    .toolbar {{
      justify-content: flex-start;
      margin-top: 10px;
    }}
    .grid {{
      grid-template-columns: 1fr;
    }}
    canvas {{
      height: 210px;
    }}
  }}
</style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>Live Body PID</h1>
      <div class="meta">{html.escape(source)}</div>
    </div>
    <div class="toolbar">
      <select id="window">
        <option value="10">10s</option>
        <option value="30" selected>30s</option>
        <option value="60">60s</option>
        <option value="0">all</option>
      </select>
      <button id="pause" type="button">Pause</button>
      <button id="clear" type="button">Clear</button>
    </div>
  </header>
  <div class="status" id="status">waiting for samples</div>
  <div class="grid">
    <section class="panel wide" data-chart="wheels">
      <div class="panel-head"><h2>Wheel Speed Setpoint vs Measured</h2><div class="legend"></div></div>
      <canvas></canvas>
    </section>
    <section class="panel" data-chart="body">
      <div class="panel-head"><h2>Body Speed Loop</h2><div class="legend"></div></div>
      <canvas></canvas>
    </section>
    <section class="panel" data-chart="turn">
      <div class="panel-head"><h2>Turn Loop</h2><div class="legend"></div></div>
      <canvas></canvas>
    </section>
    <section class="panel" data-chart="torque">
      <div class="panel-head"><h2>Torque Output</h2><div class="legend"></div></div>
      <canvas></canvas>
    </section>
    <section class="panel" data-chart="pid">
      <div class="panel-head"><h2>PID Terms</h2><div class="legend"></div></div>
      <canvas></canvas>
    </section>
  </div>
</main>
<script>
const maxRows = {max_rows};
let rows = [];
let nextSeq = 0;
let paused = false;
let lastPollMs = 0;

const colors = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9"];
const charts = {{
  wheels: {{
    lines: [
      ["wheel_l_desired", "left setpoint"],
      ["wheel_l_measured", "left measured"],
      ["wheel_r_desired", "right setpoint"],
      ["wheel_r_measured", "right measured"],
    ],
  }},
  body: {{
    lines: [
      ["speed_desired", "setpoint"],
      ["speed_measured", "measured"],
      ["speed_error", "error"],
    ],
  }},
  turn: {{
    lines: [
      ["speed_diff_desired", "setpoint"],
      ["speed_diff_measured", "measured"],
      ["turn_error", "error"],
    ],
  }},
  torque: {{
    lines: [
      ["torque_l_raw", "left raw"],
      ["torque_l_cmd", "left CAN"],
      ["torque_r_raw", "right raw"],
      ["torque_r_cmd", "right CAN"],
    ],
  }},
  pid: {{
    lines: [
      ["speed_pid_p", "speed P"],
      ["speed_pid_i", "speed I"],
      ["turn_pid_p", "turn P"],
      ["turn_pid_i", "turn I"],
    ],
  }},
}};

function number(row, key) {{
  const value = Number(row[key]);
  return Number.isFinite(value) ? value : NaN;
}}

function sampleTime(row) {{
  const t = number(row, "t");
  if (Number.isFinite(t)) return t;
  return number(row, "frame") * 0.01;
}}

function visibleRows() {{
  const seconds = Number(document.querySelector("#window").value);
  if (seconds <= 0 || rows.length === 0) return rows;
  const latest = sampleTime(rows[rows.length - 1]);
  return rows.filter((row) => sampleTime(row) >= latest - seconds);
}}

function bounds(values) {{
  const finite = values.filter(Number.isFinite);
  if (finite.length === 0) return [-1, 1];
  let low = Math.min(...finite);
  let high = Math.max(...finite);
  if (low === high) {{
    const span = Math.max(Math.abs(low) * 0.1, 1);
    return [low - span, high + span];
  }}
  const span = high - low;
  return [low - span * 0.06, high + span * 0.06];
}}

function tickLabel(value) {{
  const abs = Math.abs(value);
  if (abs >= 100) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(1);
  return value.toFixed(2);
}}

function drawChart(section, spec, data) {{
  const canvas = section.querySelector("canvas");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));

  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);

  const pad = {{left: 52, right: 16, top: 14, bottom: 30}};
  const plotW = rect.width - pad.left - pad.right;
  const plotH = rect.height - pad.top - pad.bottom;

  ctx.font = "12px -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif";
  ctx.lineWidth = 1;
  ctx.strokeStyle = "#e5e9ee";
  ctx.fillStyle = "#66717d";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";

  const xs = data.map(sampleTime);
  const allY = [];
  for (const [key] of spec.lines) {{
    for (const row of data) allY.push(number(row, key));
  }}

  const xb = bounds(xs);
  const yb = bounds(allY);
  if (xb[0] === xb[1]) {{
    xb[0] -= 1;
    xb[1] += 1;
  }}

  for (let i = 0; i <= 4; i += 1) {{
    const frac = i / 4;
    const y = pad.top + frac * plotH;
    const value = yb[1] - frac * (yb[1] - yb[0]);
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(rect.width - pad.right, y);
    ctx.stroke();
    ctx.fillText(tickLabel(value), pad.left - 8, y);
  }}

  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let i = 0; i <= 5; i += 1) {{
    const frac = i / 5;
    const x = pad.left + frac * plotW;
    const value = xb[0] + frac * (xb[1] - xb[0]);
    ctx.beginPath();
    ctx.moveTo(x, pad.top);
    ctx.lineTo(x, rect.height - pad.bottom);
    ctx.stroke();
    ctx.fillText(tickLabel(value - xb[0]), x, rect.height - pad.bottom + 8);
  }}

  ctx.strokeStyle = "#778391";
  ctx.beginPath();
  ctx.moveTo(pad.left, pad.top);
  ctx.lineTo(pad.left, rect.height - pad.bottom);
  ctx.lineTo(rect.width - pad.right, rect.height - pad.bottom);
  ctx.stroke();

  function px(x) {{
    return pad.left + ((x - xb[0]) / (xb[1] - xb[0])) * plotW;
  }}
  function py(y) {{
    return pad.top + (1 - ((y - yb[0]) / (yb[1] - yb[0]))) * plotH;
  }}

  spec.lines.forEach(([key], idx) => {{
    ctx.strokeStyle = colors[idx % colors.length];
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    for (const row of data) {{
      const x = sampleTime(row);
      const y = number(row, key);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      if (!started) {{
        ctx.moveTo(px(x), py(y));
        started = true;
      }} else {{
        ctx.lineTo(px(x), py(y));
      }}
    }}
    ctx.stroke();
  }});
}}

function renderLegends() {{
  for (const [id, spec] of Object.entries(charts)) {{
    const section = document.querySelector(`[data-chart="${{id}}"]`);
    section.querySelector(".legend").innerHTML = spec.lines.map(([_, label], idx) => (
      `<span><i class="swatch" style="background:${{colors[idx % colors.length]}}"></i>${{label}}</span>`
    )).join("");
  }}
}}

function render() {{
  const data = visibleRows();
  for (const [id, spec] of Object.entries(charts)) {{
    drawChart(document.querySelector(`[data-chart="${{id}}"]`), spec, data);
  }}

  const status = document.querySelector("#status");
  if (rows.length === 0) {{
    status.textContent = "waiting for samples";
    return;
  }}
  const latest = rows[rows.length - 1];
  const enabled = Number(latest.enabled) ? "enabled" : "disabled";
  const lag = lastPollMs ? `${{((Date.now() - lastPollMs) / 1000).toFixed(1)}}s ago` : "never";
  status.textContent = `${{rows.length}} samples buffered | frame ${{latest.frame}} | ${{enabled}} | last update ${{lag}}`;
}}

async function poll() {{
  if (paused) return;
  try {{
    const response = await fetch(`/data?since=${{nextSeq}}&max_rows=${{maxRows}}`, {{cache: "no-store"}});
    const payload = await response.json();
    if (payload.rows.length) {{
      rows.push(...payload.rows);
      if (rows.length > maxRows) rows = rows.slice(rows.length - maxRows);
    }}
    nextSeq = payload.next;
    lastPollMs = Date.now();
    render();
  }} catch (err) {{
    document.querySelector("#status").textContent = `dashboard connection error: ${{err}}`;
  }}
}}

document.querySelector("#pause").addEventListener("click", (event) => {{
  paused = !paused;
  event.target.textContent = paused ? "Resume" : "Pause";
  event.target.classList.toggle("active", paused);
}});
document.querySelector("#clear").addEventListener("click", () => {{
  rows = [];
  nextSeq = 0;
  render();
}});
document.querySelector("#window").addEventListener("change", render);
window.addEventListener("resize", render);

renderLegends();
setInterval(poll, 250);
setInterval(render, 1000);
poll();
</script>
</body>
</html>
"""


class BodyDebugServer:
  def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, max_rows: int = DEFAULT_MAX_ROWS):
    self.host = host
    self.max_rows = max_rows
    self.lock = threading.Lock()
    self.rows = deque(maxlen=max_rows)
    self.seq = 0
    self.stopped = False

    self.server = DaemonThreadingHTTPServer((host, port), self._make_handler())
    self.thread = threading.Thread(target=self.server.serve_forever, name="body-debug-server", daemon=True)
    self.thread.start()
    atexit.register(self.stop)
    print(f"body debug server: {_display_url(host, self.server.server_port)}", flush=True)

  def stop(self):
    if self.stopped:
      return
    self.stopped = True
    self.server.shutdown()
    self.server.server_close()

  def append(self, row):
    json_row = {key: _json_value(value) for key, value in row.items()}
    with self.lock:
      json_row["_seq"] = self.seq
      self.rows.append(json_row)
      self.seq += 1

  def snapshot_since(self, since: int, max_rows: int):
    with self.lock:
      rows = [row for row in self.rows if row["_seq"] >= since]
      if len(rows) > max_rows:
        rows = rows[-max_rows:]
      return {
        "rows": rows,
        "next": self.seq,
      }

  def _make_handler(self):
    debug_server = self

    class Handler(BaseHTTPRequestHandler):
      def log_message(self, fmt, *args):
        return

      def send_bytes(self, status: int, body: bytes, content_type: str):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

      def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
          body = dashboard_html("CarController in-memory debug stream", debug_server.max_rows).encode("utf-8")
          self.send_bytes(200, body, "text/html; charset=utf-8")
          return

        if parsed.path == "/data":
          query = parse_qs(parsed.query)
          try:
            since = int(query.get("since", ["0"])[0])
            row_limit = int(query.get("max_rows", [str(debug_server.max_rows)])[0])
          except ValueError:
            since = 0
            row_limit = debug_server.max_rows
          row_limit = max(1, min(row_limit, debug_server.max_rows))
          payload = debug_server.snapshot_since(since, row_limit)
          body = json.dumps(payload, allow_nan=False).encode("utf-8")
          self.send_bytes(200, body, "application/json; charset=utf-8")
          return

        self.send_bytes(404, b"not found\n", "text/plain; charset=utf-8")

    return Handler
