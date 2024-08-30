#!/usr/bin/env python3
import io
import time
import base64
import argparse
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
from collections import defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path

from opendbc.car.structs import CarControl
from opendbc.car.panda_runner import PandaRunner

DT = 0.01  # step time (s)

class Setup(Enum):
  STOPPED = 0
  STEADY_STATE_SPEED = 1

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
  setup: Setup         # initial state
  actions: list[Action]

  def get_msgs(self):
    t = 0
    for action in self.actions:
      for lt, msg in action.get_msgs():
        t += lt
        yield t, msg

MANEUVERS = [
  Maneuver(
    "creeping: alternate between +1m/ss and -1m/ss",
    Setup.STOPPED,
    [
      Action(1, 1), Action(-1, 1),
      Action(1, 1), Action(-1, 1),
      Action(1, 1), Action(-1, 1),
    ],
  ),
  # Maneuver(
  #   "brake step response: 20mph steady state -> -1m/ss",
  #   Setup.STEADY_STATE_SPEED,
  # ),
  # Maneuver(
  #   "brake step response: 20mph steady state -> max brake",
  #   Setup.STEADY_STATE_SPEED,
  # ),
  # Maneuver(
  #   "gas step response: 20mph steady state -> +1m/ss",
  #   Setup.STEADY_STATE_SPEED,
  # ),
  # Maneuver(
  #   "gas step response: 20mph steady state -> max gas",
  #   Setup.STEADY_STATE_SPEED,
  # ),
]

def main(args):
  with PandaRunner() as p:
    print("\n\n")

    logs = {}
    for i, m in enumerate(MANEUVERS):
      print(f"Running {i+1}/{len(MANEUVERS)} '{m.description}'")

      log = defaultdict(list)
      logs[m.description] = log

      # cleanup and get into a good state
      print("- setting up")
      good_cnt = 0
      for _ in range(int(30./DT)):
        cs = p.read()

        cc = CarControl(enabled=True)
        if m.setup == Setup.STOPPED:
          cc.longActive = True
          cc.actuators.accel = -1.5
          cc.actuators.longControlState = CarControl.Actuators.LongControlState.stopping
          good_cnt = (good_cnt+1) if cs.vEgo < 0.1 and cs.cruiseState.enabled and not cs.cruiseState.standstill else 0

        if not p.panda.health()['controls_allowed']:
          cc = CarControl(enabled=False)
        p.write(cc)

        if good_cnt > (2./DT):
          break
        time.sleep(DT)
      else:
        print("ERROR: failed to setup")
        continue

      # run the maneuver
      print("- executing maneuver")
      for t, cc in m.get_msgs():
        cs = p.read()
        p.send(cc)

        log["t"].append(t)
        to_log = {"carControl": cc, "carState": cs, "carControl.actuators": cc.actuators,
                  "carControl.cruiseControl": cc.cruiseControl, "carState.cruiseState": cs.cruiseState}
        for k, v in to_log.items():
          for k2, v2 in asdict(v).items():
            log[f"{k}.{k2}"].append(v2)

        time.sleep(DT)

  # ***** write out report *****

  def plt2html():
    plt.legend()
    plt.tight_layout(pad=0)
    buffer = io.BytesIO()
    plt.savefig(buffer, format='png')
    buffer.seek(0)
    return f"<img src='data:image/png;base64,{base64.b64encode(buffer.getvalue()).decode()}' style='width:100%; max-width:800px;'>\n"

  output_path = Path(__file__).resolve().parent / "longitudinal_reports"
  #output_fn = output_path / f"{p.CI.CP.carFingerprint}_{time.strftime('%Y%m%d-%H_%M_%S')}.html"
  output_fn = output_path / f"{p.CI.CP.carFingerprint}.html"
  output_path.mkdir(exist_ok=True)
  with open(output_fn, "w") as f:
    f.write("<h1>Longitudinal maneuver report</h1>\n")
    f.write(f"<h3>{p.CI.CP.carFingerprint}</h3>\n")
    if args.desc:
      f.write(f"<h3>{args.desc}</h3>")
    for m in MANEUVERS:
      f.write("<div style='border-top: 1px solid #000; margin: 20px 0;'></div>\n")
      f.write(f"<h2>{m.description}</h2>\n")

      log = logs[m.description]

      # accel plot
      plt.figure(figsize=(12, 4))
      plt.plot(log["t"], log["carState.aEgo"], label='aEgo')
      plt.plot(log["t"], log["carControl.actuators.accel"], label='actuators.accel')
      plt.xlabel('Time (s)')
      plt.ylabel('Acceleration (m/s^2)')
      plt.ylim(-2.2, 2.2)
      plt.title('Acceleration Profile')
      plt.grid(True)
      f.write(plt2html())
      plt.close()

      # secondary plot
      for k in ("carControl.enabled", "carState.cruiseState.enabled"):
        plt.rcParams['lines.linewidth'] = 2
        plt.figure(figsize=(12, 1))
        plt.plot(log["t"], log[k], label=k)
        plt.ylim(0.1, 1.1)
        # plt.grid(False)
        # plt.axis('off')
        plt.ylabel('  ')   # for alignment
        f.write(plt2html())
        plt.close()

  print(f"\nReport written to {output_fn.relative_to(Path(__file__).parent)}\n")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="A tool for longitudinal control testing.",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--desc', help="Extra description to include in report.")
  args = parser.parse_args()

  main(args)
