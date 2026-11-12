#!/usr/bin/env python3
# replays the test routes' CAN from a DUT's UP station into its panda, and checks the panda's safety
# decisions match libsafety's for the same frames. run on a rack DUT with `up run`
import os
import sys
import time
from collections import Counter

from opendbc.car.car_helpers import interfaces
from opendbc.car.tests.routes import routes
from opendbc.car.tests.test_models import NUM_JOBS, JOB_ID, TestCarModelBase
from opendbc.safety.tests.libsafety import libsafety_py

SECONDS = float(os.environ.get("REPLAY_SECONDS", "20"))
# below 1 the route plays slower than it was driven, the decisions checked here don't depend on timing
SPEED = float(os.environ.get("REPLAY_SPEED", "1"))
DT = 0.01  # the station sends a list of frames every 10 ms
BUSES = 3


def load(route):
  # the same segment, frames and safety config as test_models
  R = type("R", (TestCarModelBase,), {"test_route": route, "platform": route.car_model})
  car_fw, can_msgs, alpha_long = R.get_testing_data()
  CP = interfaces[R.platform].get_params(R.platform, R.fingerprint, car_fw, alpha_long, False, docs=False)
  cfg = CP.safetyConfigs[-1]

  t0 = can_msgs[0][0]
  frames = [[] for _ in range(int(SECONDS / DT / SPEED))]
  for t, msgs in can_msgs:
    i = int((t - t0) / 1e9 / DT / SPEED)
    if i >= len(frames):
      break
    frames[i] += [(m.address, bytes(m.dat), m.src % 4) for m in msgs if m.src < 64 and m.src % 4 < BUSES]
  return cfg.safetyModel.raw, cfg.safetyParam, frames


def expected(mode, param, frames):
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(mode, param) == 0
  safety.set_alternative_experience(0)
  rx, fwd, invalid = Counter(), Counter(), 0
  for i, frame in enumerate(frames):
    safety.set_timer(int(i * DT * 1e6))
    # like the panda's 1 Hz tick
    if i % int(1 / DT) == 0:
      safety.safety_tick()
    for addr, dat, bus in frame:
      fwd[bus] += safety.safety_fwd_hook(bus, addr) >= 0
      invalid += not safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
      rx[bus] += 1
  return {"rx": [rx[b] for b in range(BUSES)], "fwd": [fwd[b] for b in range(BUSES)], "rx_invalid": invalid,
          "relay_malfunction": bool(safety.get_relay_malfunction())}


def played(p, up, mode, param, frames):
  # like pandad, a heartbeat alone doesn't end the power save that loading the route without one started
  p.set_power_save(0)
  p.set_safety_mode(mode, param)
  # can_health is per CAN controller, a flipped harness swaps the controllers of buses 0 and 2
  cans = [2, 1, 0] if p.health()["car_harness_status"] == p.HARNESS_STATUS_FLIPPED else [0, 1, 2]
  h0 = [p.can_health(c) for c in cans]
  # an empty tail, so the station's loop doesn't start over before it's stopped
  up.can_replay(frames + [[]] * int(60 / DT))
  end = time.monotonic() + len(frames) * DT + 1
  while time.monotonic() < end:
    p.send_heartbeat(engaged=False)
    time.sleep(0.2)
  h, h1 = p.health(), [p.can_health(c) for c in cans]
  up.reset()
  up.set_ignition(True)
  assert h["safety_mode"] == mode, f"panda left safety mode {mode}: {h['safety_mode']}"
  return {"rx": [h1[b]["total_rx_cnt"] - h0[b]["total_rx_cnt"] for b in range(BUSES)],
          "fwd": [h1[b]["total_fwd_cnt"] - h0[b]["total_fwd_cnt"] for b in range(BUSES)],
          "rx_invalid": h["safety_rx_invalid"], "relay_malfunction": bool(h["faults"] & 1)}


def main():
  from panda import Panda
  from ultimate_provisioning import UP

  os.environ["SAFETY_UBSAN_TRAP"] = "1"
  p, up = Panda(), UP.remote()
  # the station starts with the harness lines floating, reset() puts it in like played() leaves it
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
    except Exception as e:
      print(f"{route.route} FAIL, can't load: {type(e).__name__}: {e!r}", flush=True)
      failed.append(route.route)
      continue
    if any(len(dat) > 8 for frame in frames for _, dat, _ in frame):
      # the station only sends classic CAN
      print(f"{route.route} skipped, CAN FD", flush=True)
      continue
    exp, got = expected(mode, param, frames), played(p, up, mode, param, frames)
    # forwarding stops at a relay malfunction, when depends on the 1 Hz tick's phase
    if exp["relay_malfunction"]:
      del exp["fwd"], got["fwd"]
    ok = exp == got
    # a frame the station didn't get on the bus also makes the decisions differ
    dropped = [e - g for e, g in zip(exp["rx"], got["rx"], strict=True)]
    print(f"{route.route} {route.car_model} mode {mode} {'ok' if ok else 'FAIL'} {time.monotonic() - st:.0f}s expected {exp} panda {got}" +
          (f" station dropped {dropped}" if any(dropped) else ""), flush=True)
    if not ok:
      failed.append(route.route)

  up.reset()
  print(f"{len(failed)} failed: {failed}")
  return int(bool(failed))


if __name__ == "__main__":
  sys.exit(main())
