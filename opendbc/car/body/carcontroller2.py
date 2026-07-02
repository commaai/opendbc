import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.body import bodycan
from opendbc.car.body.debug_server import BodyDebugServer
from opendbc.car.body.values import SPEED_FROM_RPM
from opendbc.car.interfaces import CarControllerBase

MAX_TORQUE = 500
KP = 120


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    try:
      self.debug_server = BodyDebugServer()
    except OSError as e:
      self.debug_server = None
      print(f"body debug server failed to start: {e}", flush=True)

  def _debug_log(self, now_nanos, CC, speed_desired, speed_measured, speed_error,
                 speed_diff_desired, speed_diff_measured, turn_error,
                 wheel_l_desired, wheel_r_desired, wheel_l_measured, wheel_r_measured,
                 torque_l_raw, torque_r_raw, torque_l_cmd, torque_r_cmd):
    if self.debug_server is None:
      return

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
      "speed_pid_p": KP * speed_error,
      "speed_pid_i": 0.,
      "speed_pid_output": KP * speed_error,
      "turn_pid_p": KP * turn_error,
      "turn_pid_i": 0.,
      "turn_pid_output": KP * turn_error,
      "torque_l_raw": torque_l_raw,
      "torque_r_raw": torque_r_raw,
      "torque_l_cmd": torque_l_cmd,
      "torque_r_cmd": torque_r_cmd,
    }

    self.debug_server.append(row)

  def update(self, CC, CS, now_nanos):
    torque_l = 0
    torque_r = 0
    torque_l_raw = 0.
    torque_r_raw = 0.

    speed_desired = 0.
    speed_diff_desired = 0.

    wheel_l_measured = SPEED_FROM_RPM * CS.out.wheelSpeeds.fl
    wheel_r_measured = SPEED_FROM_RPM * CS.out.wheelSpeeds.fr
    speed_measured = (wheel_l_measured + wheel_r_measured) / 2.
    speed_diff_measured = (wheel_r_measured - wheel_l_measured) / self.CP.wheelbase
    speed_error = 0.
    turn_error = 0.

    wheel_l_desired = 0.
    wheel_r_desired = 0.

    if CC.enabled:
      # Read these from the joystick
      # TODO: this isn't acceleration, okay?
      speed_desired = CC.actuators.accel / 4. # 1 * y m/s
      speed_diff_desired = CC.actuators.torque * 4 # 1 * x m/s

      # desired wheel linear speeds (m/s)
      wheel_r_desired = speed_desired + speed_diff_desired * self.CP.wheelbase / 2
      wheel_l_desired = speed_desired - speed_diff_desired * self.CP.wheelbase / 2

      speed_error = speed_desired - speed_measured
      turn_error = speed_diff_desired - speed_diff_measured

      # proportional torque on velocity error
      torque = KP * speed_error
      torque_diff = KP * turn_error * self.CP.wheelbase / 2

      # Bias the turn onto the inner wheel: hold the outer wheel at the forward
      # command and take the whole speed difference out of the inner wheel,
      # instead of splitting it symmetrically (which pushes the outer wheel
      # above the forward command).
      turn = abs(torque_diff)
      common = np.sign(torque) * max(0., abs(torque) - turn)
      torque_r_raw = common + torque_diff
      torque_l_raw = common - torque_diff

      torque_r = int(np.clip(torque_r_raw, -MAX_TORQUE, MAX_TORQUE))
      torque_l = int(np.clip(torque_l_raw, -MAX_TORQUE, MAX_TORQUE))

    self._debug_log(now_nanos, CC, speed_desired, speed_measured, speed_error,
                    speed_diff_desired, speed_diff_measured, turn_error,
                    wheel_l_desired, wheel_r_desired, wheel_l_measured, wheel_r_measured,
                    torque_l_raw, torque_r_raw, torque_l, torque_r)

    can_sends = []
    can_sends.append(bodycan.create_control(self.packer, torque_l, torque_r))

    new_actuators = CC.actuators.as_builder()
    new_actuators.accel = torque_l
    new_actuators.torque = torque_r
    new_actuators.torqueOutputCan = torque_r

    self.frame += 1
    return new_actuators, can_sends
