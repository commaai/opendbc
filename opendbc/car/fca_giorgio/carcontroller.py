from opendbc.car import Bus
from opendbc.can.packer import CANPacker
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.fca_giorgio import fca_giorgiocan
from opendbc.car.fca_giorgio.values import CanBus, CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0
    self.frame = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      can_sends.append(fca_giorgiocan.create_steering_control(self.packer_pt, CanBus.pt, apply_torque, CC.latActive))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.HUD_1_STEP == 0:
      can_sends.append(fca_giorgiocan.create_lka_hud_1_control(self.packer_pt, CanBus.pt, CC.latActive))
    if self.frame % self.CCP.HUD_2_STEP == 0:
      can_sends.append(fca_giorgiocan.create_lka_hud_2_control(self.packer_pt, CanBus.pt, CC.latActive))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
