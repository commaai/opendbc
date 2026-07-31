"""REPRO ONLY. Check MebCreepChurnRepro without driving the car.

  replay <segment>   feed a recorded route's carState/carControl through the harness at 50 Hz and
                     report what it would put on the wire. Real vEgo / esp_hold / brake / gas /
                     enabled traces, so every arming gate and legality check is exercised.
  sim [v0] [grade]   closed loop against a crude plant, which is the only way to see the creep and
                     the churn, because in replay vEgo answers to whatever build recorded the route.

Both run the emissions through the real MebLongStateMachine, so an illegal HMS transition here is
one master could produce too.
"""
import sys
from collections import Counter
from types import SimpleNamespace as NS

from opendbc.can import CANParser
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.mebcan import MebFaultReplay, MebLongStateMachine
from opendbc.car.volkswagen.values import CarControllerParams, VolkswagenFlags

HMS = {0: "none", 1: "HALTEN", 2: "PARKEN", 3: "stby", 4: "ANFAHREN", 5: "RAMP"}
DT = 0.02

CP = CarParams(carFingerprint="VOLKSWAGEN_ID4_MK2", flags=int(VolkswagenFlags.MEB | VolkswagenFlags.MEB_GEN2))
CCP = CarControllerParams(CP)


def step(repro, long_state, v, held, brake=False, gas=False, enabled=True):
  CS = NS(out=NS(accFaulted=False, vEgo=v, brakePressed=brake, gasPressed=gas, cruiseState=NS(available=True)),
          esp_hold_confirmation=held)
  CC = NS(enabled=enabled, longActive=enabled and not gas, cruiseControl=NS(override=gas),
          actuators=NS(longControlState=0, accel=0.0))
  accel, status, hold_type, braking_to_stop = long_state.update(CS, CC, 0.0)
  accel, hold_type, braking_to_stop = repro.update(CS, CC, accel, status, hold_type, braking_to_stop)
  long_state.prev_acc_hold_type = hold_type
  return accel, hold_type, braking_to_stop, repro.frame is not None


def report(out):
  """out rows: (t, accel, hold_type, braking_to_stop, v, held)"""
  trans = sum(1 for a, b in zip(out, out[1:], strict=False) if a[2] != b[2])
  released_while_held = [r for r in out if r[5] and r[2] == 0]
  grabs = sum(1 for a, b in zip(out, out[1:], strict=False) if not a[5] and b[5])
  releases = sum(1 for a, b in zip(out, out[1:], strict=False) if a[5] and not b[5])
  span = max(1e-3, out[-1][0] - out[0][0])
  print(f"  HMS values emitted   : {dict(Counter(HMS[r[2]] for r in out))}")
  print(f"  HMS transitions      : {trans} over {span:.1f} s = {trans / span:.1f}/s  (b6 phase B was 6.5/s)")
  print(f"  accel range          : {min(r[1] for r in out):.2f} .. {max(r[1] for r in out):.2f}")
  print(f"  hold grabbed/released: {grabs}x / {releases}x")
  print(f"  no-request while held: {len(released_while_held)}   <-- must be 0, this is what clamped the EPB into park in 000000ca")


def replay(seg):
  from openpilot.tools.lib.logreader import LogReader
  repro, long_state = MebFaultReplay(CP, CCP), MebLongStateMachine(CP, CCP)
  car = CANParser("vw_meb_2024_generated", [("ESC_50", 50)], 0)
  v, brake, gas, enabled, t0, frame, steps, out = 0.0, False, False, False, None, 0, 0, []
  for m in LogReader(seg):
    if m.which() == "carState":
      v, brake, gas = m.carState.vEgo, m.carState.brakePressed, m.carState.gasPressed
    elif m.which() == "carControl":
      enabled = m.carControl.enabled
    elif m.which() == "can":
      t0 = m.logMonoTime if t0 is None else t0
      car.update((m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can]))
      frame += 1
      if frame % 2:  # the harness runs at 50 Hz, CAN arrives at 100
        continue
      held = bool(car.vl["ESC_50"]["Standstill"])
      accel, hold_type, braking_to_stop, is_armed = step(repro, long_state, v, held, brake, gas, enabled)
      if is_armed:
        out.append(((m.logMonoTime - t0) / 1e9, round(accel, 2), hold_type, braking_to_stop, round(v, 2), held))
      steps += 1

  armed = out
  print(f"{seg}: {steps} steps, {len(armed)} under the harness ({100 * len(armed) / max(1, steps):.0f}%)")
  if armed:
    report(armed)


def sim(v0=5.0, grade=0.0, seconds=30.0, release_frames=5, release_accel=0.5):
  """Crude plant: an integrator, plus an ESP hold that grabs when stopped under a hold request and
  only lets go after release_frames of a sustained drive-off request worth at least release_accel.
  That threshold is the thing the harness has to beat and nobody has measured it, so it is
  deliberately pessimistic: b6 poked the hold with 0.12 m/s^2 for 0.21 s and it never let go, while
  0000007f/30 released within a frame of ANFAHREN + 1.51. Enough to show the creep and the churn,
  not a car model. It has no creep torque, so it needs more commanded accel to move than a real car.
  """
  repro, long_state = MebFaultReplay(CP, CCP), MebLongStateMachine(CP, CCP)
  v, held, release_delay, out = v0, False, 0, []
  for i in range(int(seconds / DT)):
    accel, hold_type, braking_to_stop, _ = step(repro, long_state, v, held)
    if held:
      v = 0.0
      asking = hold_type not in (1, 2) and accel >= release_accel
      release_delay = release_delay + 1 if asking else 0
      if release_delay >= release_frames:
        held, release_delay = False, 0
    else:
      v = max(0.0, v + (accel - grade) * DT)
      held = v < 0.05 and braking_to_stop
    out.append((i * DT, round(accel, 2), hold_type, braking_to_stop, round(v, 3), held))

  settled = [r for r in out if r[0] > 5.0]
  print(f"sim: engage at {v0} m/s on a {grade:.2f} m/s^2 grade, {seconds:.0f} s")
  report(out)
  lo, hi = min(r[4] for r in settled), max(r[4] for r in settled)
  print(f"  speed after settling : {lo:.2f} .. {hi:.2f} m/s   (b6 crept at 0.06-0.12)")
  print("\n  t     accel  HMS       hld  v")
  last = None
  for r in out:
    if 5.0 < r[0] < 11.0 and (r[2], r[5]) != last:
      print(f"  {r[0]:5.2f} {r[1]:6.2f}  {HMS[r[2]]:8s}  {int(r[5])}    {r[4]:.3f}")
      last = (r[2], r[5])


if __name__ == "__main__":
  if sys.argv[1] == "sim":
    sim(*[float(a) for a in sys.argv[2:]])
  else:
    replay(sys.argv[1])
