#!/usr/bin/env python3
# replays the test routes into the panda from the rack's UP and compares its safety decisions with libsafety's
import os
import sys
import time
from collections import Counter

from opendbc.car.car_helpers import interfaces
from opendbc.car.tests.routes import routes
from opendbc.car.tests.test_models import NUM_JOBS, JOB_ID, TestCarModelBase
from opendbc.safety.tests.libsafety import libsafety_py

SECONDS = float(os.environ.get("REPLAY_SECONDS", "20"))
DT = 0.01
BUSES = 3
# seconds until the RX checks are compared
STEADY = 2


def load(route):
  R = type("R", (TestCarModelBase,), {"test_route": route, "platform": route.car_model})
  car_fw, can_msgs, alpha_long = R.get_testing_data()
  CP = interfaces[R.platform].get_params(R.platform, R.fingerprint, car_fw, alpha_long, False, docs=False)
  cfg = CP.safetyConfigs[-1]

  t0 = can_msgs[0][0]
  frames = [[] for _ in range(int(SECONDS / DT))]
  for t, msgs in can_msgs:
    i = int((t - t0) / 1e9 / DT)
    if i >= len(frames):
      break
    frames[i] += [(m.address, bytes(m.dat), m.src % 4) for m in msgs if m.src < 64 and m.src % 4 < BUSES]
  return cfg.safetyModel.raw, cfg.safetyParam, frames


def without_forwarded_copies(mode, param, frames):
  # some logs also have the panda's forwarded copy, replaying it would collide with the panda's own forward
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(mode, param) == 0
  out = []
  for frame in frames:
    copies = Counter(frame)
    drop = Counter()
    for (addr, dat, bus), n in copies.items():
      if bus == 0 and (addr, dat, 2) in copies and safety.safety_fwd_hook(0, addr) == 2:
        drop[(addr, dat, 2)] += min(n, copies[(addr, dat, 2)])
      elif bus == 2 and (addr, dat, 0) in copies and safety.safety_fwd_hook(2, addr) == 0 and safety.safety_fwd_hook(0, addr) != 2:
        drop[(addr, dat, 0)] += min(n, copies[(addr, dat, 0)])
    kept = []
    for msg in frame:
      if drop[msg]:
        drop[msg] -= 1
      else:
        kept.append(msg)
    out.append(kept)
  return out


def expected(mode, param, frames):
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(mode, param) == 0
  safety.set_alternative_experience(0)
  rx, fwd, invalid, checks_invalid = Counter(), Counter(), 0, False
  for i, frame in enumerate(frames):
    safety.set_timer(int(i * DT * 1e6))
    if i % int(1 / DT) == 0:
      safety.safety_tick()
      checks_invalid |= i * DT >= STEADY and bool(safety.safety_a.state.safety_rx_checks_invalid)
    for addr, dat, bus in frame:
      fwd[bus] += safety.safety_fwd_hook(bus, addr) >= 0
      invalid += not safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
      rx[bus] += 1
  return {"rx": [rx[b] for b in range(BUSES)], "fwd": [fwd[b] for b in range(BUSES)], "rx_invalid": invalid,
          "relay_malfunction": bool(safety.get_relay_malfunction()), "rx_checks_invalid": checks_invalid,
          "safety": [mode, param], "faults": 0, "tx_buffer_overflow": 0, "heartbeat_disabled": False, "harness_changed": False}


def within(lo, hi, got):
  if isinstance(lo, list):
    return all(within(a, b, g) for a, b, g in zip(lo, hi, got, strict=True))
  return min(lo, hi) <= got <= max(lo, hi)


def played(p, up, mode, param, frames):
  # loading the route without a heartbeat puts the panda in power save
  p.set_power_save(0)
  p.set_safety_mode(mode, param)
  # can_health is per CAN controller, a flipped harness swaps buses 0 and 2
  cans = [2, 1, 0] if p.health()["car_harness_status"] == p.HARNESS_STATUS_FLIPPED else [0, 1, 2]
  hs0, h0 = p.health(), [p.can_health(c) for c in cans]
  fd = {bus for frame in frames for _, dat, bus in frame if len(dat) > 8}
  # the panda forwards CAN FD frames between buses 0 and 2
  if fd & {0, 2}:
    fd |= {0, 2}
  for b in fd:
    up.set_can_data_speed_kbps(b, 2000)
  # an empty tail, so the station's loop doesn't start over
  up.can_replay(frames + [[]] * int(60 / DT))
  start, checks_invalid = time.monotonic(), False
  while time.monotonic() < start + len(frames) * DT + 1:
    p.send_heartbeat(engaged=False)
    # the panda updates its RX checks once a second
    if STEADY + 1 <= time.monotonic() - start <= len(frames) * DT:
      checks_invalid |= p.health()["safety_rx_checks_invalid"]
    time.sleep(0.2)
  h, h1 = p.health(), [p.can_health(c) for c in cans]
  up.reset()
  up.set_ignition(True)
  return {"rx": [h1[b]["total_rx_cnt"] - h0[b]["total_rx_cnt"] for b in range(BUSES)],
          "fwd": [h1[b]["total_fwd_cnt"] - h0[b]["total_fwd_cnt"] for b in range(BUSES)],
          "rx_invalid": h["safety_rx_invalid"], "relay_malfunction": bool(h["faults"] & 1), "rx_checks_invalid": checks_invalid,
          "safety": [h["safety_mode"], h["safety_param"]],
          "faults": h["faults"] & ~hs0["faults"] & ~1,
          # only TX, the queue to the host overflows without pandad
          "tx_buffer_overflow": h["tx_buffer_overflow"] - hs0["tx_buffer_overflow"],
          "heartbeat_disabled": h["heartbeat_disabled"], "harness_changed": h["car_harness_status"] != hs0["car_harness_status"],
          # only printed, to debug a failure
          "can_errors": {k: [h1[b][k] - h0[b][k] for b in range(BUSES)] for k in ("total_error_cnt", "total_tx_lost_cnt", "bus_off_cnt", "can_core_reset_count")}}


def main():
  from panda import Panda
  from ultimate_provisioning import UP

  os.environ["SAFETY_UBSAN_TRAP"] = "1"
  p, up = Panda(), UP.remote()
  # the station starts with the harness lines floating
  up.reset()
  up.set_ignition(True)

  names = sys.argv[1:]
  failed = []
  for i, route in enumerate(routes):
    if i % NUM_JOBS != JOB_ID or (names and route.route not in names):
      continue
    st = time.monotonic()
    try:
      mode, param, frames = load(route)
      frames = without_forwarded_copies(mode, param, frames)
    except Exception as e:
      print(f"{route.route} FAIL, can't load: {type(e).__name__}: {e!r}", flush=True)
      failed.append(route.route)
      continue
    # the station doesn't control the order of one 10 ms list's frames on different buses
    lo, hi = (expected(mode, param, [sorted(f, key=lambda m: m[2] * order) for f in frames]) for order in (1, -1))
    got = played(p, up, mode, param, frames)
    # forwarding stops at a relay malfunction, when depends on the tick
    if lo["relay_malfunction"] or hi["relay_malfunction"]:
      for d in (lo, hi, got):
        del d["fwd"]
    ok = all(within(lo[k], hi[k], got[k]) for k in lo)
    exp = lo if lo == hi else {"low": lo, "high": hi}
    dropped = [e - g for e, g in zip(lo["rx"], got["rx"], strict=True)]
    print(f"{route.route} {route.car_model} mode {mode} {'ok' if ok else 'FAIL'} {time.monotonic() - st:.0f}s expected {exp} panda {got}" +
          (f" station dropped {dropped}" if any(dropped) else ""), flush=True)
    if not ok:
      failed.append(route.route)

  up.reset()
  print(f"{len(failed)} failed: {failed}")
  return int(bool(failed))


if __name__ == "__main__":
  sys.exit(main())
