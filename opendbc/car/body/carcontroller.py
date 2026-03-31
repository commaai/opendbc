import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.common.pid import PIDController
from opendbc.car.body import bodycan
from opendbc.car.body.values import CAR, SPEED_FROM_RPM
from opendbc.car.interfaces import CarControllerBase

MAX_TORQUE = 700
MAX_TORQUE_RATE = 70
MAX_ANGLE_ERROR = np.radians(7)
MAX_POS_INTEGRATOR = 0.2
MAX_TURN_INTEGRATOR = 0.2

class BodyCarController(CarControllerBase):

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.control_bus = 0

    # PIDs
    self.v_pid = PIDController(110, k_i=11.5, rate=1 / DT_CTRL)
    self.w_pid = PIDController(110, k_i=11.5, rate=1 / DT_CTRL)

    self.torque_r_filtered = 0.
    self.torque_l_filtered = 0.

  def update(self, CC, CS, now_nanos):

    torque_l = 0
    torque_r = 0

    if CC.enabled:
      v_setpoint = CC.actuators.accel / 4.
      w_setpoint = CC.actuators.torque

      # deadzones
      if abs(v_setpoint) < 0.05:
        v_setpoint = 0.
      if abs(w_setpoint) < 0.05:
        w_setpoint = 0.

      v_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl + CS.out.wheelSpeeds.fr) / 2.
      v_error = v_setpoint - v_measured
      freeze_v_integrator = ((v_error < 0 and self.v_pid.error_integral <= -MAX_POS_INTEGRATOR) or
                            (v_error > 0 and self.v_pid.error_integral >= MAX_POS_INTEGRATOR))
      v_torque = self.v_pid.update(v_error, freeze_integrator=freeze_v_integrator)

      w_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl - CS.out.wheelSpeeds.fr) / 2.
      w_error = w_setpoint - w_measured
      freeze_w_integrator = ((w_error < 0 and self.w_pid.error_integral <= -MAX_POS_INTEGRATOR) or
                            (w_error > 0 and self.w_pid.error_integral >= MAX_POS_INTEGRATOR))
      w_torque = self.w_pid.update(w_error, freeze_integrator=freeze_w_integrator)

      torque_r = v_torque + w_torque
      torque_l = v_torque - w_torque

      # Torque rate limits
      self.torque_r_filtered = np.clip(torque_r,
                                       self.torque_r_filtered - MAX_TORQUE_RATE,
                                       self.torque_r_filtered + MAX_TORQUE_RATE)
      self.torque_l_filtered = np.clip(torque_l,
                                       self.torque_l_filtered - MAX_TORQUE_RATE,
                                       self.torque_l_filtered + MAX_TORQUE_RATE)
      torque_r = int(np.clip(self.torque_r_filtered, -MAX_TORQUE, MAX_TORQUE))
      torque_l = int(np.clip(self.torque_l_filtered, -MAX_TORQUE, MAX_TORQUE))

    can_sends = []
    can_sends.append(bodycan.create_control(self.packer, self.control_bus, torque_l, torque_r))

    new_actuators = CC.actuators.as_builder()
    new_actuators.accel = torque_l
    new_actuators.torque = torque_r
    new_actuators.torqueOutputCan = torque_r

    self.frame += 1
    return new_actuators, can_sends

class BodyV2(BodyCarController):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.control_bus = 2

class CarController(CarControllerBase):
  def __new__(cls, dbc_names, CP):
    if CP.carFingerprint == CAR.COMMA_BODY_V2:
      return BodyV2(dbc_names, CP)
    return BodyCarController(dbc_names, CP)