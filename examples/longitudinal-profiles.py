#!/usr/bin/env python3
import io
import time
import base64
import argparse
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path

from opendbc.car.structs import CarControl
from opendbc.car.panda_runner import PandaRunner
from opendbc.car.common.conversions import Conversions

DT = 0.01  # step time (s)

# TODOs
# - support lateral maneuvers

@dataclass
class Action:
  accel: float      # m/s^2
  duration: float   # seconds
  longControlState: CarControl.Actuators.LongControlState = CarControl.Actuators.LongControlState.pid

  def get_msgs(self):
    return [
      (t, CarControl(
        enabled=True,
        longActive=True,
        actuators=CarControl.Actuators(
          accel=self.accel,
          longControlState=self.longControlState,
        ),
      ))
      for t in np.linspace(0, self.duration, int(self.duration/DT))
    ]

@dataclass
class Maneuver:
  description: str
  actions: list[Action]
  repeat: int = 1
  initial_speed: float = 0.  # m/s

  def get_msgs(self):
    t0 = 0
    for action in self.actions:
      for lt, msg in action.get_msgs():
        yield lt + t0, msg
      t0 += lt

MANEUVERS = [
  Maneuver(
    "creep: alternate between +1m/ss and -1m/ss",
    [
      Action(1, 2), Action(-1, 2),
      Action(1, 2), Action(-1, 2),
      Action(1, 2), Action(-1, 2),
    ],
    initial_speed=0.,
  ),
  Maneuver(
    "brake step response: -1m/ss from 20mph",
    [Action(-1, 3),],
    repeat=3,
    initial_speed=20. * Conversions.MPH_TO_MS,
  ),
  Maneuver(
    "brake step response: -4m/ss from 20mph",
    [Action(-4, 3),],
    repeat=3,
    initial_speed=20. * Conversions.MPH_TO_MS,
  ),
  Maneuver(
    "gas step response: +1m/ss from 20mph",
    [Action(1, 3),],
    repeat=3,
    initial_speed=20. * Conversions.MPH_TO_MS,
  ),
  Maneuver(
    "gas step response: +4m/ss from 20mph",
    [Action(4, 3),],
    repeat=3,
    initial_speed=20. * Conversions.MPH_TO_MS,
  ),
]

def main(args):
  with PandaRunner() as p:
    print("\n\n")

    logs = {}
    for i, m in enumerate(MANEUVERS):
      print(f"Running {i+1}/{len(MANEUVERS)} '{m.description}'")
      for run in range(0, m.repeat):
        print(f"- run #{run}")
        print("- setting up, engage cruise")
        ready_cnt = 0
        for _ in range(int(60./DT)):
          cs = p.read(strict=False)

          cc = CarControl(
            enabled=True,
            longActive=True,
            actuators=CarControl.Actuators(
              accel=(m.initial_speed - cs.vEgo)*0.5,
              longControlState=CarControl.Actuators.LongControlState.stopping if m.initial_speed < 0.1 else CarControl.Actuators.LongControlState.pid,
            ),
          )
          #print(target, ready_cnt, cs.vEgo, cc.actuators)
          p.write(cc)

          ready = cs.cruiseState.enabled and not cs.cruiseState.standstill and ((m.initial_speed - 0.5) < cs.vEgo < (m.initial_speed + 0.5))
          ready_cnt = (ready_cnt+1) if ready else 0
          if ready_cnt > (2./DT):
            break
          time.sleep(DT)
        else:
          print("ERROR: failed to setup")
          continue

        print("- executing maneuver")
        logs[m.description + f" #{run}"] = defaultdict(list)
        for t, cc in m.get_msgs():
          cs = p.read()
          p.write(cc)

          logs[m.description + f" #{run}"]["t"].append(t)
          to_log = {"carControl": cc, "carState": cs, "carControl.actuators": cc.actuators,
                    "carControl.cruiseControl": cc.cruiseControl, "carState.cruiseState": cs.cruiseState}
          for k, v in to_log.items():
            for k2, v2 in asdict(v).items():
              logs[m.description + f" #{run}"][f"{k}.{k2}"].append(v2)

          time.sleep(DT)

  # ***** write out report *****
  output_path = Path(__file__).resolve().parent / "longitudinal_reports"
  output_fn = args.output or output_path / f"{p.CI.CP.carFingerprint}_{time.strftime('%Y%m%d-%H_%M_%S')}.html"
  output_path.mkdir(exist_ok=True)
  with open(output_fn, "w") as f:
    f.write("<h1>Longitudinal maneuver report</h1>\n")
    f.write(f"<h3>{p.CI.CP.carFingerprint}</h3>\n")
    if args.desc:
      f.write(f"<h3>{args.desc}</h3>")
    for description, log in logs.items():
      f.write("<div style='border-top: 1px solid #000; margin: 20px 0;'></div>\n")
      f.write(f"<h2>{description}</h2>\n")

      plt.rcParams['font.size'] = 40
      fig = plt.figure(figsize=(30, 20))
      ax = fig.subplots(3, 1, sharex=True, gridspec_kw={'hspace': 0, 'height_ratios': [5, 1, 1]})

      ax[0].grid(linewidth=4)
      ax[0].plot(log["t"], log["carState.aEgo"], label='aEgo', linewidth=6)
      ax[0].plot(log["t"], log["carControl.actuators.accel"], label='accel command', linewidth=6)
      ax[0].set_ylabel('Acceleration (m/s^2)')
      ax[0].set_ylim(-4.5, 4.5)
      ax[0].legend()

      ax[1].plot(log["t"], log["carControl.enabled"], label='enabled', linewidth=6)
      ax[2].plot(log["t"], log["carState.gasPressed"], label='gasPressed', linewidth=6)
      for i in (1, 2):
        ax[i].set_yticks([0, 1], minor=False)
        ax[i].set_ylim(-1, 2)
        ax[i].legend()

      ax[-1].set_xlabel("Time (s)")
      fig.tight_layout()

      buffer = io.BytesIO()
      fig.savefig(buffer, format='png')
      buffer.seek(0)
      f.write(f"<img src='data:image/png;base64,{base64.b64encode(buffer.getvalue()).decode()}' style='width:100%; max-width:800px;'>\n")

  print(f"\nReport written to {output_fn}\n")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="A tool for longitudinal control testing.",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--desc', help="Extra description to include in report.")
  parser.add_argument('--output', help="Write out report to this file.", default=None)
  args = parser.parse_args()

  assert args.output is None or args.output.endswith(".html"), "Output filename must end with '.html'"

  main(args)
