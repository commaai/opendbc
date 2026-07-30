"""Replay a real route through MebHoldPulseRepro and report what it would emit.

Feeds recorded carState/carControl into the harness at the real 50 Hz control rate, so every
arming gate, counter and phase transition is exercised against real vEgo / esp_hold / brake /
enabled traces. Catches the whole class of bugs that previously only showed up on the car.
"""
import sys
from collections import Counter
from types import SimpleNamespace as NS

from opendbc.can import CANParser
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.mebcan import MebHoldPulseRepro
from opendbc.car.volkswagen.values import CarControllerParams, VolkswagenFlags
from openpilot.tools.lib.logreader import LogReader

HMS = {0: "none", 1: "HALTEN", 2: "PARKEN", 3: "stby", 4: "ANFAHREN", 5: "RAMP"}
SEG = sys.argv[1]

CP = CarParams(carFingerprint="VOLKSWAGEN_ID4_MK2", flags=int(VolkswagenFlags.MEB | VolkswagenFlags.MEB_GEN2))
CCP = CarControllerParams(CP)
repro = MebHoldPulseRepro(CP, CCP)

car = CANParser("vw_meb_2024_generated", [("ESC_50", 50)], 0)
v = 0.0
brake = False
enabled = False
t0 = None
step = 0
out = []
for m in LogReader(SEG):
  w = m.which()
  if w == "carState":
    v, brake = m.carState.vEgo, m.carState.brakePressed
    continue
  if w == "carControl":
    enabled = m.carControl.enabled
    continue
  if w != "can":
    continue
  if t0 is None:
    t0 = m.logMonoTime
  car.update((m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can]))
  step += 1
  if step % 2:            # harness runs at 50 Hz, CAN frames arrive at 100 Hz
    continue
  hold = bool(car.vl["ESC_50"]["Standstill"])
  CS = NS(out=NS(accFaulted=False, vEgo=v, brakePressed=brake, cruiseState=NS(available=True)),
          esp_hold_confirmation=hold)
  CC = NS(longActive=enabled, enabled=enabled, cruiseControl=NS(override=False),
          actuators=NS(longControlState=0, accel=0.0))
  # sentinels: if the harness passes these through, it handed the policy back
  a, h, b = repro.update(CS, CC, -99.0, -99, False)
  out.append(((m.logMonoTime - t0) / 1e9, round(a, 2), h, b, round(v, 2), hold, enabled))

armed = [r for r in out if r[1] != -99.0]
pulses = sum(1 for a, b in zip(out, out[1:], strict=False) if b[2] == 4 and a[2] != 4)
moving_pulse = sum(1 for r in out if r[2] == 4 and r[4] > 0.1)
pct = 100 * len(armed) / max(1, len(out))
name = SEG.split('/')[-2][:8] + '/' + SEG.split('/')[-1]
print(f"{name}: {len(out)} control steps, {len(armed)} under harness control ({pct:.0f}%)")
print(f"  ANFAHREN pulses emitted        : {pulses}")
print(f"  pulses while vEgo > 0.1 m/s    : {moving_pulse}   <-- must be 0, would drive the car")
print(f"  HMS values emitted while armed : {dict(Counter(HMS.get(r[2], r[2]) for r in armed))}")
print(f"  accels emitted while armed     : {dict(Counter(r[1] for r in armed))}")
if armed:
  runs, cur = [], 1
  for a, b in zip(armed, armed[1:], strict=False):
    if b[0] - a[0] < 0.05:
      cur += 1
    else:
      runs.append(cur)
      cur = 1
  runs.append(cur)
  print(f"  longest continuous control run : {max(runs) * 0.02:.1f} s over {len(runs)} activations")
