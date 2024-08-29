#!/usr/bin/env python3
import io
import time
import base64
import argparse
import matplotlib.pyplot as plt
from enum import Enum
from collections import defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path

from opendbc.car.structs import CarControl, CarState
from opendbc.car.can_definitions import CanData
from opendbc.car.panda_runner import PandaRunner

DT = 0.01  # step time (s)
OUTPUT_PATH = Path(__file__).resolve().parent / "longitudinal_reports"

class Setup(Enum):
  STOPPED = 0
  STEADY_STATE_SPEED = 1

@dataclass
class Maneuver:
  description: str
  setup: Setup         # initial state

  _log = defaultdict(list)

  def get_cc(self, t: float) -> CarControl:
    CC = CarControl(
      enabled=True,
      latActive=False,
      longActive=True,

      actuators=CarControl.Actuators(
        gas=0.,
        brake=0.,
        speed=0.,
        accel=0.,
        longControlState=CarControl.Actuators.LongControlState.off,
      ),

      cruiseControl=CarControl.CruiseControl(
        cancel=False,
        resume=False,
        override=False,
      ),
    )
    return CC

  def get_msgs(self):
    for t in range(0, int(1./DT)):
      yield t, self.get_cc(t)

  def log(self, t, cc: CarControl, cs: CarState) -> None:
    self._log["t"].append(t)
    to_log = {"carControl": cc, "carState": cs, "carControl.actuators": cc.actuators, "carControl.cruiseControl": cc.cruiseControl, "carState.cruiseState": cs.cruiseState}
    for k, v in to_log.items():
      for k2, v2 in asdict(v).items():
        self._log[f"{k}.{k2}"].append(v2)

MANEUVERS = [
  Maneuver(
    "start from stop",
    Setup.STOPPED,
  ),
  Maneuver(
    "brake step response: 20mph steady state -> -1m/ss",
    Setup.STEADY_STATE_SPEED,
  ),
  Maneuver(
    "brake step response: 20mph steady state -> max brake",
    Setup.STEADY_STATE_SPEED,
  ),
  Maneuver(
    "gas step response: 20mph steady state -> +1m/ss",
    Setup.STEADY_STATE_SPEED,
  ),
  Maneuver(
    "gas step response: 20mph steady state -> max gas",
    Setup.STEADY_STATE_SPEED,
  ),
  Maneuver(
    "creeping: alternate between +1m/ss and -1m/ss",
    Setup.STOPPED,
  ),
]

def main(args):
  with PandaRunner() as (p, CI):
    print("\n\n")

    for i, m in enumerate(MANEUVERS):
      print(f"Running {i+1}/{len(MANEUVERS)} '{m.description}'")

      # cleanup and get into a good state
      print("- setting up")
      cs = None
      for _ in range(int(3./DT)):
        cd = [CanData(addr, dat, bus) for addr, dat, bus in p.can_recv()]
        cs = CI.update([0, cd])
        _, can_sends = CI.apply(CarControl(enabled=False))
        p.can_send_many(can_sends, timeout=1000)
        time.sleep(DT)
      #assert not cs.cruiseState.enabled, "Cruise control not disabled"

      # run the maneuver
      print("- executing maneuver")
      for t, msg in m.get_msgs():
        cd = [CanData(addr, dat, bus) for addr, dat, bus in p.can_recv()]
        cs = CI.update([0, cd])
        #assert cs.canValid, f"CAN went invalid, check connections"

        _, can_sends = CI.apply(msg)
        #p.can_send_many(can_sends, timeout=20)

        m.log(t, msg, cs)
        time.sleep(DT)

        if len(m._log["t"]) > 100:
          break

  # ***** write out report *****

  def plt2html():
    plt.legend()
    plt.tight_layout(pad=0)

    buffer = io.BytesIO()
    plt.savefig(buffer, format='png')
    buffer.seek(0)
    return f"<img src='data:image/png;base64,{base64.b64encode(buffer.getvalue()).decode()}' style='width:100%; max-width:800px;'>\n"

  output_fn = OUTPUT_PATH / f"{CI.CP.carFingerprint}_{time.strftime('%Y%m%d-%H_%M_%S')}.html"
  OUTPUT_PATH.mkdir(exist_ok=True)
  with open(output_fn, "w") as f:
    f.write(f"<h1>Longitudinal maneuver report</h1>\n")
    f.write(f"<h3>{CI.CP.carFingerprint}</h3>\n")
    if args.desc:
      f.write(f"<h3>{args.desc}</h3>")
    for m in MANEUVERS:
      f.write(f"<div style='border-top: 1px solid #000; margin: 20px 0;'></div>\n")
      f.write(f"<h2>{m.description}</h2>\n")

      # accel plot
      plt.figure(figsize=(12, 4))
      plt.plot(m._log["t"], m._log["carState.aEgo"], label='aEgo')
      plt.plot(m._log["t"], m._log["carControl.actuators.accel"], label='actuators.accel')
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
        plt.plot(m._log["t"], m._log[k], label=k)
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