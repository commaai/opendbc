import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.common.pid import PIDController
from opendbc.car.body import bodycan
from opendbc.car.body.values import CAR, SPEED_FROM_RPM
from opendbc.car.interfaces import CarControllerBase

MAX_TORQUE = 1000
MAX_TORQUE_RATE = 250
ACCEL_INPUT_MAX = 4.0

MAX_VELOCITY = 3.0
MAX_TURN_RATE = 2.0

ACCEL_LIMIT = 1.0
DECEL_LIMIT = 100000.0
TURN_ACCEL_LIMIT = 3.0
TURN_DECEL_LIMIT = 100000.0

KV = 100
KS = 10

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.control_bus = 2 if self.CP.carFingerprint == CAR.COMMA_BODY_V2 else 0

    self.v_pid = PIDController(k_p=1000, k_i=100, rate=1 / DT_CTRL)
    self.w_pid = PIDController(k_p=2000, k_i=500, rate=1 / DT_CTRL)

    self.v_setpoint = 0.0
    self.w_setpoint = 0.0

    self.torque_r_filtered = 0.
    self.torque_l_filtered = 0.

  def update(self, CC, CS, now_nanos):

    torque_l = 0
    torque_r = 0

    if CC.enabled:
      v_target = (CC.actuators.accel / ACCEL_INPUT_MAX) * MAX_VELOCITY
      w_target = -CC.actuators.torque * MAX_TURN_RATE

      user_wants_to_move = (abs(v_target) > 0.01 or abs(w_target) > 0.01)
      robot_is_stopped = (abs(self.v_setpoint) < 0.05 and abs(self.w_setpoint) < 0.05)

      if not user_wants_to_move and robot_is_stopped:
        self.v_setpoint = 0
        self.w_setpoint = 0
        self.v_pid.reset()
        self.w_pid.reset()
      else:
        is_accelerating = abs(v_target) > abs(self.v_setpoint)
        if is_accelerating:
          step = ACCEL_LIMIT * DT_CTRL
        else:
          step = DECEL_LIMIT * DT_CTRL
        v_error = v_target - self.v_setpoint
        self.v_setpoint += np.clip(v_error, -step, step)
        self.v_setpoint = np.clip(self.v_setpoint, -MAX_VELOCITY, MAX_VELOCITY)

        is_accelerating = abs(w_target) > abs(self.w_setpoint)
        if is_accelerating:
          w_step = TURN_ACCEL_LIMIT * DT_CTRL
        else:
          w_step = TURN_DECEL_LIMIT * DT_CTRL
        w_error = w_target - self.w_setpoint
        self.w_setpoint += np.clip(w_error, -w_step, w_step)
        self.w_setpoint = np.clip(self.w_setpoint, -MAX_TURN_RATE, MAX_TURN_RATE)

        speed_l = -1 * SPEED_FROM_RPM * CS.out.wheelSpeeds.fl
        speed_r = SPEED_FROM_RPM * CS.out.wheelSpeeds.fr

        v_actual = (speed_l + speed_r) / 2.0
        w_actual = (speed_r - speed_l) / 0.3

        v_ff = (self.v_setpoint * KV) + (np.sign(self.v_setpoint) * KS) if abs(self.v_setpoint) > 0.01 else 0

        t_linear = self.v_pid.update(self.v_setpoint - v_actual, freeze_integrator=False) + v_ff
        t_angular = self.w_pid.update(self.w_setpoint - w_actual, freeze_integrator=False)

        # Combine base torque with steering differential
        torque_r = t_linear + t_angular
        torque_l = t_linear - t_angular
        max_requested = max(abs(torque_r), abs(torque_l))
        if max_requested > MAX_TORQUE:
          torque_l = torque_l * (MAX_TORQUE / max_requested)
          torque_r = torque_r * (MAX_TORQUE / max_requested)

        # Torque rate limits
        self.torque_r_filtered = np.clip(torque_r,
                                         self.torque_r_filtered - MAX_TORQUE_RATE,
                                         self.torque_r_filtered + MAX_TORQUE_RATE)
        self.torque_l_filtered = np.clip(torque_l,
                                         self.torque_l_filtered - MAX_TORQUE_RATE,
                                         self.torque_l_filtered + MAX_TORQUE_RATE)
        torque_r = int(np.clip(self.torque_r_filtered, -MAX_TORQUE, MAX_TORQUE))
        torque_l = int(np.clip(self.torque_l_filtered, -MAX_TORQUE, MAX_TORQUE))
        print(f"{torque_l=}, {torque_r=}, {CC.actuators.accel=}, {CC.actuators.torque=} {self.v_setpoint=} {self.w_setpoint=} {speed_l=} {speed_r=}")
    else:
      self.torque_r_filtered = 0.
      self.torque_l_filtered = 0.
      self.v_pid.reset()
      self.w_pid.reset()
      self.v_setpoint = 0
      self.w_setpoint = 0

    can_sends = []
    can_sends.append(bodycan.create_control(self.packer, self.control_bus, torque_l, torque_r))

    new_actuators = CC.actuators.as_builder()
    new_actuators.accel = torque_l
    new_actuators.torque = torque_r / MAX_TORQUE
    new_actuators.torqueOutputCan = torque_r

    self.frame += 1
    return new_actuators, can_sends
