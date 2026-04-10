import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.common.pid import PIDController
from opendbc.car.body import bodycan
from opendbc.car.body.values import SPEED_FROM_RPM
from opendbc.car.interfaces import CarControllerBase

MAX_SPEED = 1 # m/s
MAX_TURN = 1 # m/s
MAX_POS_INTEGRATOR = 1
MAX_TORQUE = 700
MAX_TORQUE_RATE = 70


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])

    self.v_pid_settings = {
      "k_p": 0.6 * MAX_TORQUE_RATE,
      "k_i": 0.6 * MAX_TORQUE_RATE,
      "k_d": 0.0 * MAX_TORQUE_RATE,
    }
    self.w_pid_settings = {
        "k_p": [[0, MAX_SPEED], [0.6 * MAX_TORQUE_RATE, 0.1 * MAX_TORQUE_RATE]],
        "k_i": [[0, MAX_SPEED], [0.6 * MAX_TORQUE_RATE, 0.1 * MAX_TORQUE_RATE]],
        "k_d": [[0, MAX_SPEED], [0.0 * MAX_TORQUE_RATE, 0.0 * MAX_TORQUE_RATE]],
    }

    self.v_pid = PIDController(**self.v_pid_settings, rate=1 / DT_CTRL)
    self.w_pid = PIDController(**self.w_pid_settings, rate=1 / DT_CTRL)

    self.torque_r_filtered = 0.
    self.torque_l_filtered = 0.

  def update(self, CC, CS, now_nanos):
    torque_l = 0
    torque_r = 0

    if CC.enabled:
      v_setpoint = -(CC.actuators.accel / 4.0) * MAX_SPEED
      w_setpoint = -CC.actuators.torque * MAX_TURN

      user_wants_to_move = (abs(w_setpoint) > 0.01 or abs(v_setpoint) > 0.01)
      robot_is_stopped = (abs(v_setpoint) < 0.05 and abs(w_setpoint) < 0.05)

      if not user_wants_to_move and robot_is_stopped:
        self.v_setpoint = 0
        self.w_setpoint = 0
        self.v_pid.reset()
        self.w_pid.reset()

      v_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl + CS.out.wheelSpeeds.fr) / 2.

      # remove wind down on left/right
      if abs(w_setpoint) < 0.05:
        self.w_setpoint =0
        self.w_pid.reset()

      v_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl + CS.out.wheelSpeeds.fr) / 2.
      v_error = v_measured - v_setpoint
      freeze_v_integrator = ((v_error < 0 and self.v_pid.error_integral <= -MAX_POS_INTEGRATOR) or
                            (v_error > 0 and self.v_pid.error_integral >= MAX_POS_INTEGRATOR))
      v_torque = self.v_pid.update(v_error, freeze_integrator=freeze_v_integrator)

      w_measured = SPEED_FROM_RPM * -(CS.out.wheelSpeeds.fl - CS.out.wheelSpeeds.fr)
      w_error = w_measured - w_setpoint
      freeze_w_integrator = ((w_error < 0 and self.w_pid.error_integral <= -MAX_POS_INTEGRATOR) or
                            (w_error > 0 and self.w_pid.error_integral >= MAX_POS_INTEGRATOR))
      w_torque = self.w_pid.update(w_error, freeze_integrator=freeze_w_integrator)

      torque_r = v_torque + w_torque
      torque_l = v_torque - w_torque

      self.torque_r_filtered = np.clip(torque_r,
                                       self.torque_r_filtered - MAX_TORQUE_RATE,
                                       self.torque_r_filtered + MAX_TORQUE_RATE)
      self.torque_l_filtered = np.clip(torque_l,
                                       self.torque_l_filtered - MAX_TORQUE_RATE,
                                       self.torque_l_filtered + MAX_TORQUE_RATE)
      torque_r = int(np.clip(self.torque_r_filtered, -MAX_TORQUE, MAX_TORQUE))
      torque_l = int(np.clip(self.torque_l_filtered, -MAX_TORQUE, MAX_TORQUE))

    can_sends = []
    can_sends.append(bodycan.create_control(self.packer, torque_l, torque_r))

    new_actuators = CC.actuators.as_builder()
    new_actuators.accel = torque_l
    new_actuators.torque = torque_r
    new_actuators.torqueOutputCan = torque_r

    self.frame += 1
    return new_actuators, can_sends