#!/usr/bin/env python3
# replays the car test routes through libsafety
#   dump DIR             each route's CAN in 10 ms steps, with openpilot's TX
#   digest DIR OUT       every RX and TX result and the safety state, hashed per route and second
#   compare BASE NEW     which routes and seconds differ
# digest only uses libsafety, so another opendbc can digest with this file:
#   PYTHONPATH=<other opendbc> python route_replay.py digest DIR OUT
import argparse
import hashlib
import json
import multiprocessing
import pickle
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

DT = 0.01
BUSES = 3
# a TX message's bus, like cereal's can src
TX = 128
STATE = ("get_controls_allowed", "get_relay_malfunction", "get_gas_pressed_prev", "get_brake_pressed_prev", "get_regen_braking_prev",
         "get_steering_disengage_prev", "get_cruise_engaged_prev", "get_vehicle_moving", "get_acc_main_on", "get_torque_meas_min",
         "get_torque_meas_max", "get_torque_driver_min", "get_torque_driver_max", "get_angle_meas_min", "get_angle_meas_max",
         "get_curvature_meas_min", "get_curvature_meas_max")


def load(route, seconds: float | None = None, tx: bool = False):
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.tests.test_models import TestCarModelBase

  R = type("R", (TestCarModelBase,), {"test_route": route, "platform": route.car_model})
  car_fw, can_msgs, alpha_long = R.get_testing_data(route)
  CP = interfaces[R.platform].get_params(R.platform, R.fingerprint, car_fw, alpha_long, False, docs=False)
  cfg = CP.safetyConfigs[-1]

  t0 = can_msgs[0][0]
  steps = int((can_msgs[-1][0] - t0) / 1e9 / DT) + 1 if seconds is None else int(seconds / DT)
  frames = [[] for _ in range(steps)]
  for t, msgs in can_msgs:
    i = int((t - t0) / 1e9 / DT)
    if i >= len(frames):
      break
    for m in msgs:
      if m.src < 64 and m.src % 4 < BUSES:
        frames[i].append((m.address, bytes(m.dat), m.src % 4))
      elif tx and TX <= m.src < TX + 64 and m.src % 4 < BUSES:
        frames[i].append((m.address, bytes(m.dat), TX + m.src % 4))
  return cfg.safetyModel.raw, cfg.safetyParam, frames


def replay(mode: int, param: int, frames, on_tick=None):
  # the result of each message and the safety state after each step
  from opendbc.safety.tests.libsafety import libsafety_py
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(mode, param) == 0
  safety.set_alternative_experience(0)
  getters = [getattr(safety, g) for g in STATE if hasattr(safety, g)]

  steps = []
  for i, frame in enumerate(frames):
    safety.set_timer(int(i * DT * 1e6))
    if i % int(1 / DT) == 0:
      safety.safety_tick()
      if on_tick is not None:
        on_tick(i, safety)
    results = []
    for addr, dat, bus in frame:
      if bus >= TX:
        results.append(bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus - TX, dat))))
      else:
        fwd = safety.safety_fwd_hook(bus, addr)
        results.append((bool(safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))), fwd))
    steps.append((results, tuple(g() for g in getters)))
  return steps


def dump_route(args):
  i, route, out = args
  try:
    mode, param, frames = load(route, tx=True)
  except Exception as e:
    return f"{route.route} can't load: {e!r}"
  with open(out / f"{i}.pkl", "wb") as f:
    pickle.dump((route.route, route.car_model, mode, param, frames), f)
  return None


def digest_route(path):
  with open(path, "rb") as f:
    route, model, mode, param, frames = pickle.load(f)
  steps = replay(mode, param, frames)
  per_second = int(1 / DT)
  seconds, sums = [], []
  for s in range(0, len(steps), per_second):
    chunk = steps[s:s + per_second]
    seconds.append(hashlib.sha256(repr(chunk).encode()).hexdigest()[:16])
    # what changed in a second that differs: valid RX, allowed TX, then each state value summed over the second
    results = [r for step_results, _ in chunk for r in step_results]
    sums.append([sum(r[0] for r in results if isinstance(r, tuple)), sum(r for r in results if not isinstance(r, tuple)),
                 *(int(sum(v)) for v in zip(*(state for _, state in chunk), strict=True))])
  rx = sum(isinstance(r, tuple) for results, _ in steps for r in results)
  tx = sum(not isinstance(r, tuple) for results, _ in steps for r in results)
  return route, {"model": model, "seconds": seconds, "sums": sums, "rx": rx, "tx": tx}


def pool(jobs: int):
  return ProcessPoolExecutor(jobs, mp_context=multiprocessing.get_context("spawn"))


def dump(out: Path, jobs: int) -> int:
  from opendbc.car.tests.routes import routes
  out.mkdir(parents=True, exist_ok=True)
  with pool(jobs) as p:
    errors = [e for e in p.map(dump_route, [(i, r, out) for i, r in enumerate(routes)]) if e]
  for e in errors:
    print(e)
  print(f"dumped {len(routes) - len(errors)} routes to {out}")
  return int(bool(errors))


def digest(dumps: Path, out: Path, jobs: int) -> int:
  from opendbc.safety.tests.libsafety import libsafety_py
  state = [g for g in STATE if hasattr(libsafety_py.libsafety, g)]
  with pool(jobs) as p:
    res = dict(p.map(digest_route, sorted(dumps.glob("*.pkl"))))
  out.write_text(json.dumps({"state": state, "routes": res}, indent=1, sort_keys=True))
  return 0


def compare(base: Path, new: Path) -> int:
  a, b = json.loads(base.read_text()), json.loads(new.read_text())
  if a["state"] != b["state"]:
    print(f"libsafety has different state getters: base {a['state']}, new {b['state']}")
    return 1
  state, a, b = a["state"], a["routes"], b["routes"]
  if a.keys() != b.keys():
    print(f"different routes: only in base {sorted(a.keys() - b.keys())}, only in new {sorted(b.keys() - a.keys())}")
    return 1
  diff = []
  for route in sorted(a):
    x, y = a[route], b[route]
    changed = [s for s, (h0, h1) in enumerate(zip(x["seconds"], y["seconds"], strict=True)) if h0 != h1]
    if changed:
      fields = ["valid RX", "allowed TX", *(g.removeprefix("get_") for g in state)]
      what = sorted({fields[k] for s in changed for k, (v0, v1) in enumerate(zip(x["sums"][s], y["sums"][s], strict=True)) if v0 != v1})
      diff.append(route)
      print(f"{route} {x['model']}: differs from {changed[0]} s on, in {len(changed)} of {len(x['seconds'])} s: {', '.join(what) or 'order only'}")
  seconds = sum(len(x["seconds"]) for x in a.values())
  rx, tx = sum(x["rx"] for x in a.values()), sum(x["tx"] for x in a.values())
  print(f"{len(a)} routes, {seconds} s, {rx} RX and {tx} TX messages: " + (f"{len(diff)} routes differ" if diff else "no difference"))
  return int(bool(diff))


def main() -> int:
  parser = argparse.ArgumentParser(description="replay the car test routes through libsafety")
  sub = parser.add_subparsers(dest="cmd", required=True)
  p = sub.add_parser("dump")
  p.add_argument("out", type=Path)
  p = sub.add_parser("digest")
  p.add_argument("dumps", type=Path)
  p.add_argument("out", type=Path)
  p = sub.add_parser("compare")
  p.add_argument("base", type=Path)
  p.add_argument("new", type=Path)
  for p in sub.choices.values():
    p.add_argument("-j", "--jobs", type=int, default=multiprocessing.cpu_count())
  args = parser.parse_args()

  if args.cmd == "dump":
    return dump(args.out, args.jobs)
  if args.cmd == "digest":
    return digest(args.dumps, args.out, args.jobs)
  return compare(args.base, args.new)


if __name__ == "__main__":
  sys.exit(main())
