import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.common.pid import PIDController
from opendbc.car.body import bodycan
from opendbc.car.body.values import SPEED_FROM_RPM
from opendbc.car.interfaces import CarControllerBase
from openpilot.common.params import Params

MAX_TORQUE = 700
MAX_TORQUE_RATE = 80
MAX_ANGLE_ERROR = np.radians(7)
MAX_POS_INTEGRATOR = 0.2   # meters
MAX_TURN_INTEGRATOR = 0.1  # meters


class CarController(CarControllerBase):
  PARAM_DEFAULTS = {
    'BodySpeedPidKp': 110.0,
    'BodySpeedPidKi': 11.5,
    'BodyTurnPidKp': 150.0,
    'BodyTurnPidKi': 15.0,
  }

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.params = Params()

    # PIDs
    self.turn_pid = PIDController(self.PARAM_DEFAULTS['BodyTurnPidKp'], k_i=self.PARAM_DEFAULTS['BodyTurnPidKi'], rate=1 / DT_CTRL)
    self.wheeled_speed_pid = PIDController(self.PARAM_DEFAULTS['BodySpeedPidKp'], k_i=self.PARAM_DEFAULTS['BodySpeedPidKi'], rate=1 / DT_CTRL)

    self.torque_r_filtered = 0.
    self.torque_l_filtered = 0.

  @staticmethod
  def deadband_filter(torque, deadband):
    if torque > 0:
      torque += deadband
    else:
      torque -= deadband
    return torque

  def update(self, CC, CS, now_nanos):

    torque_l = 0
    torque_r = 0

    if CC.enabled:
      # Read these from the joystick
      # TODO: this isn't acceleration, okay?
      speed_desired = CC.actuators.accel / 5.
      speed_diff_desired = -CC.actuators.torque / 1.5

      speed_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl + CS.out.wheelSpeeds.fr) / 2.
      speed_error = speed_desired - speed_measured

      torque = self.wheeled_speed_pid.update(speed_error, freeze_integrator=False)

      speed_diff_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl - CS.out.wheelSpeeds.fr)
      turn_error = speed_diff_measured - speed_diff_desired
      freeze_integrator = ((turn_error < 0 and self.turn_pid.error_integral <= -MAX_TURN_INTEGRATOR) or
                           (turn_error > 0 and self.turn_pid.error_integral >= MAX_TURN_INTEGRATOR))
      torque_diff = self.turn_pid.update(turn_error, freeze_integrator=freeze_integrator)

      # Combine 2 PIDs outputs
      torque_r = torque + torque_diff
      torque_l = torque - torque_diff

      # Torque rate limits
      self.torque_r_filtered = np.clip(self.deadband_filter(torque_r, 10),
                                       self.torque_r_filtered - MAX_TORQUE_RATE,
                                       self.torque_r_filtered + MAX_TORQUE_RATE)
      self.torque_l_filtered = np.clip(self.deadband_filter(torque_l, 10),
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

    # Read PID tuning params every ~1s
    if self.frame % 100 == 0:
      try:
        for key, pid, attr in [
          ('BodySpeedPidKp', self.wheeled_speed_pid, '_k_p'),
          ('BodySpeedPidKi', self.wheeled_speed_pid, '_k_i'),
          ('BodyTurnPidKp', self.turn_pid, '_k_p'),
          ('BodyTurnPidKi', self.turn_pid, '_k_i'),
        ]:
          val = self.params.get(key, return_default=True)
          if val is not None:
            setattr(pid, attr, [[0], [float(val)]])
      except Exception:
        pass

    self.frame += 1
    return new_actuators, can_sends
