import copy
from opendbc.can.packer import CANPacker
from opendbc.car import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_lka_steering, create_acm_status, create_longitudinal
from opendbc.car.rivian.values import CarControllerParams

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    self.CP = CP
    self.frame = 0
    self.apply_steer_last = 0
    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    apply_steer = 0
    if CC.latActive:
      new_steer = int(round(CC.actuators.steer * CarControllerParams.STEER_MAX))
      apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                     CS.out.steeringTorque, CarControllerParams)

    # send steering command
    self.apply_steer_last = apply_steer
    can_sends.append(create_lka_steering(self.packer,  CS.acm_lka_hba_cmd, apply_steer, CC.latActive))

    can_sends.append(create_acm_status(self.packer, CS.acm_status_cmd, CC.latActive))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = actuators.accel
      can_sends.append(create_longitudinal(self.packer, self.frame % 15, accel, CC.longActive))

    new_actuators = copy.copy(CC.actuators)
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends
