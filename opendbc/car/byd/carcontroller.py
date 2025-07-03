import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_std_steer_angle_limits, AngleRateLimit
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd.bydcan import create_can_steer_command, create_lkas_hud, create_accel_command

ECU_FAULT_ANGLE = 260 # degrees

class CarControllerParams:
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[4., 3., 2.])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[6., 4., 3.])

  def __init__(self, CP):
    pass

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, VM):
    super().__init__(dbc_names, CP)
    self.CP = CP
    self.frame = 0
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.lka_active = False

  def update(self, CC, CS, now_nanos):
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    apply_angle = CS.out.steeringAngleDeg
    # lkas user activation, cannot tie to lka_on state because it may deactivate itself
    if CS.lka_on:
      self.lka_active = True
    if not CS.lka_on and CS.lkas_rdy_btn:
      self.lka_active = False

    lat_active = enabled and self.lka_active and not CS.out.standstill
      #and not CS.out.steeringPressed and abs(CS.out.steeringAngleDeg) < ECU_FAULT_ANGLE

    if (self.frame % 2) == 0:
      if lat_active:
        apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, \
          CS.out.steeringAngleDeg, CS.out.vEgo, CarControllerParams)

        # assumption why eps fault:
        # 1. steer rate too high
        # 2. met with resistance while steering
        # 3. applied steer too far away from current steeringAngleDeg
        apply_angle = np.clip(apply_angle, CS.out.steeringAngleDeg - 10, CS.out.steeringAngleDeg + 10)

      can_sends.append(create_can_steer_command(self.packer, apply_angle, lat_active, CS.out.standstill, CS.lkas_healthy, CS.lkas_rdy_btn))
      can_sends.append(create_lkas_hud(self.packer, enabled, CS.lss_state, CS.lss_alert, CS.tsr, \
        CS.abh, CS.passthrough, CS.HMA, self.lka_active))

      if self.CP.openpilotLongitudinalControl:
        long_active = enabled and not CS.out.gasPressed
        brake_hold = CS.out.standstill and actuators.accel < 0
        can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, brake_hold))

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
