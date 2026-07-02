import atexit
import csv
import os
from pathlib import Path

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.common.pid import PIDController
from opendbc.car.body import bodycan
from opendbc.car.body.debug_server import BodyDebugServer
from opendbc.car.body.values import SPEED_FROM_RPM
from opendbc.car.interfaces import CarControllerBase

MAX_TORQUE = 500
MAX_TORQUE_RATE = 50
MAX_ANGLE_ERROR = np.radians(7)
MAX_POS_INTEGRATOR = 0.2   # meters
MAX_TURN_INTEGRATOR = 0.1  # meters
MAX_WHEEL_ACCEL = 50.      # m/s^2, reject single-frame wheel-speed jumps faster than any real wheel accel


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])

    # PIDs
    self.turn_pid = PIDController(50, k_i=40, rate=1 / DT_CTRL)
    self.wheeled_speed_pid = PIDController(50, k_i=40, rate=1 / DT_CTRL)

    self.torque_r_filtered = 0.
    self.torque_l_filtered = 0.

    # last (rate-limited) wheel speeds in m/s, for the measurement derivative check
    self.wheel_l_last = 0.
    self.wheel_r_last = 0.

    self.debug_csv = None
    self.debug_writer = None
    self.debug_server = None
    if os.getenv("BODY_CONTROLLER_DEBUG_SERVER") is not None:
      try:
        self.debug_server = BodyDebugServer()
      except OSError as e:
        print(f"body debug server failed to start: {e}", flush=True)
    debug_csv_path = os.getenv("BODY_CONTROLLER_DEBUG_CSV")
    if debug_csv_path is not None:
      path = Path(debug_csv_path).expanduser()
      path.parent.mkdir(parents=True, exist_ok=True)
      self.debug_csv = path.open("w", buffering=1, newline="", encoding="utf-8")
      self.debug_writer = csv.DictWriter(self.debug_csv, [
        "frame", "t", "now_nanos", "enabled",
        "input_accel", "input_torque",
        "speed_desired", "speed_measured", "speed_error",
        "speed_diff_desired", "speed_diff_measured", "turn_error",
        "wheel_l_desired", "wheel_r_desired", "wheel_l_measured", "wheel_r_measured",
        "speed_pid_p", "speed_pid_i", "speed_pid_output",
        "turn_pid_p", "turn_pid_i", "turn_pid_output", "turn_freeze_integrator",
        "torque_l_raw", "torque_r_raw",
        "torque_l_filtered", "torque_r_filtered",
        "torque_l_cmd", "torque_r_cmd",
      ])
      self.debug_writer.writeheader()
      atexit.register(self.debug_csv.close)

  @staticmethod
  def deadband_filter(torque, deadband):
    if torque > 0:
      return torque - deadband if torque > deadband else 0
    elif torque < 0:
      return torque + deadband if torque < -deadband else 0
    return torque

  def _debug_log(self, now_nanos, CC, speed_desired, speed_measured, speed_error,
                 speed_diff_desired, speed_diff_measured, turn_error, turn_freeze_integrator,
                 torque_l_raw, torque_r_raw, torque_l_cmd, torque_r_cmd):
    if self.debug_writer is None and self.debug_server is None:
      return

    wheel_l_measured = speed_measured + speed_diff_measured / 2.
    wheel_r_measured = speed_measured - speed_diff_measured / 2.
    wheel_l_desired = speed_desired + speed_diff_desired / 2.
    wheel_r_desired = speed_desired - speed_diff_desired / 2.

    row = {
      "frame": self.frame,
      "t": now_nanos * 1e-9,
      "now_nanos": now_nanos,
      "enabled": int(CC.enabled),
      "input_accel": CC.actuators.accel,
      "input_torque": CC.actuators.torque,
      "speed_desired": speed_desired,
      "speed_measured": speed_measured,
      "speed_error": speed_error,
      "speed_diff_desired": speed_diff_desired,
      "speed_diff_measured": speed_diff_measured,
      "turn_error": turn_error,
      "wheel_l_desired": wheel_l_desired,
      "wheel_r_desired": wheel_r_desired,
      "wheel_l_measured": wheel_l_measured,
      "wheel_r_measured": wheel_r_measured,
      "speed_pid_p": self.wheeled_speed_pid.p,
      "speed_pid_i": self.wheeled_speed_pid.i,
      "speed_pid_output": self.wheeled_speed_pid.control,
      "turn_pid_p": self.turn_pid.p,
      "turn_pid_i": self.turn_pid.i,
      "turn_pid_output": self.turn_pid.control,
      "turn_freeze_integrator": int(turn_freeze_integrator),
      "torque_l_raw": torque_l_raw,
      "torque_r_raw": torque_r_raw,
      "torque_l_filtered": self.torque_l_filtered,
      "torque_r_filtered": self.torque_r_filtered,
      "torque_l_cmd": torque_l_cmd,
      "torque_r_cmd": torque_r_cmd,
    }

    if self.debug_writer is not None:
      self.debug_writer.writerow(row)
    if self.debug_server is not None:
      self.debug_server.append(row)
    if self.debug_csv is not None and self.frame % 100 == 0:
      self.debug_csv.flush()

  def update(self, CC, CS, now_nanos):

    torque_l = 0
    torque_r = 0
    torque_l_raw = 0.
    torque_r_raw = 0.
    turn_freeze_integrator = False

    speed_desired = 0.
    speed_diff_desired = 0.
    # Slew-rate limit each wheel speed: a real wheel can't change faster than MAX_WHEEL_ACCEL,
    # so single-frame jumps beyond that (e.g. motor-controller transients during a hard reversal)
    # are clipped before they reach the PIDs and blow up the proportional term.
    max_wheel_delta = MAX_WHEEL_ACCEL * DT_CTRL
    wheel_l = float(np.clip(SPEED_FROM_RPM * CS.out.wheelSpeeds.fl,
                            self.wheel_l_last - max_wheel_delta, self.wheel_l_last + max_wheel_delta))
    wheel_r = float(np.clip(SPEED_FROM_RPM * CS.out.wheelSpeeds.fr,
                            self.wheel_r_last - max_wheel_delta, self.wheel_r_last + max_wheel_delta))
    self.wheel_l_last = wheel_l
    self.wheel_r_last = wheel_r

    speed_measured = (wheel_l + wheel_r) / 2.
    speed_diff_measured = wheel_l - wheel_r
    speed_error = 0.
    turn_error = 0.

    if CC.enabled:
      # Read these from the joystick
      # TODO: this isn't acceleration, okay?
      speed_desired = (CC.actuators.accel / 4.) * 2 # 1 * y m/s
      speed_diff_desired = -CC.actuators.torque * 2 # 1 * x m/s

      speed_error = speed_desired - speed_measured

      speed_freeze_integrator = ((speed_error < 0 and self.wheeled_speed_pid.error_integral <= -MAX_POS_INTEGRATOR) or
                                 (speed_error > 0 and self.wheeled_speed_pid.error_integral >= MAX_POS_INTEGRATOR))
      torque = self.wheeled_speed_pid.update(speed_error, freeze_integrator=speed_freeze_integrator)

      turn_error = speed_diff_measured - speed_diff_desired
      turn_freeze_integrator = ((turn_error < 0 and self.turn_pid.error_integral <= -MAX_TURN_INTEGRATOR) or
                                (turn_error > 0 and self.turn_pid.error_integral >= MAX_TURN_INTEGRATOR))
      torque_diff = self.turn_pid.update(turn_error, freeze_integrator=turn_freeze_integrator)

      # Bias the turn onto the inner wheel: hold the outer wheel at the forward
      # command and take the whole speed difference out of the inner wheel, instead
      # of splitting it symmetrically about `torque` (which pushes the outer wheel
      # above the forward command).
      turn = abs(torque_diff)
      common = np.sign(torque) * max(0., abs(torque) - turn)
      torque_r_raw = common + torque_diff
      torque_l_raw = common - torque_diff

      # Torque rate limits
      self.torque_r_filtered = np.clip(self.deadband_filter(torque_r_raw, 10),
                                       self.torque_r_filtered - MAX_TORQUE_RATE,
                                       self.torque_r_filtered + MAX_TORQUE_RATE)
      self.torque_l_filtered = np.clip(self.deadband_filter(torque_l_raw, 10),
                                       self.torque_l_filtered - MAX_TORQUE_RATE,
                                       self.torque_l_filtered + MAX_TORQUE_RATE)
      torque_r = int(np.clip(self.torque_r_filtered, -MAX_TORQUE, MAX_TORQUE))
      torque_l = int(np.clip(self.torque_l_filtered, -MAX_TORQUE, MAX_TORQUE))

    self._debug_log(now_nanos, CC, speed_desired, speed_measured, speed_error,
                    speed_diff_desired, speed_diff_measured, turn_error, turn_freeze_integrator,
                    torque_l_raw, torque_r_raw, torque_l, torque_r)

    can_sends = []
    can_sends.append(bodycan.create_control(self.packer, torque_l, torque_r))

    new_actuators = CC.actuators.as_builder()
    new_actuators.accel = torque_l
    new_actuators.torque = torque_r
    new_actuators.torqueOutputCan = torque_r

    self.frame += 1
    return new_actuators, can_sends
