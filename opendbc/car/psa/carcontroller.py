import numpy as np
from opendbc.car import structs, apply_std_steer_angle_limits, Bus
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa import psacan
from opendbc.car.psa.values import CarControllerParams

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    self.CP = CP
    self.packer = CANPacker(dbc_names[Bus.cam])
    self.frame = 0
    self.apply_angle_last = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    ### lateral control ###
    if CC.latActive:
      desired_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CarControllerParams)
      apply_angle = float(np.clip(desired_angle, -CarControllerParams.STEER_MAX, CarControllerParams.STEER_MAX))
    else:
      apply_angle = CS.out.steeringAngleDeg

    can_sends.append(psacan.create_lka_msg(self.packer, self.CP, self.frame, CC.latActive, apply_angle))

    self.apply_angle_last = apply_angle

    ### cruise buttons ###
    # TODO: find cruise buttons msg
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
