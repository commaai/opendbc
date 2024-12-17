from opendbc.car import apply_driver_steer_torque_limits, common_fault_avoidance, structs
from opendbc.can.packer import CANPacker
from opendbc.car.common.numpy_fast import clip
from opendbc.car import apply_std_steer_angle_limits, Bus
from opendbc.car.interfaces  import CarControllerBase
from opendbc.car.psa import psacan
from opendbc.car.psa.values import CarControllerParams

GearShifter = structs.CarState.GearShifter

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    self.CP = CP
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.lkas_max_torque = 0
    self.apply_steer_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    reverse = CS.out.gearShifter == GearShifter.reverse

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    if not CC.latActive:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    # can_sends.append(psacan.create_lka_msg(self.packer, self.CP, apply_angle, self.frame, CC.latActive, self.lkas_max_torque, reverse))
    # TODO: this copies the original lkas message
    can_sends.append(psacan.create_lka_msg(self.packer, self.CP, CS.original_lka_values))


    ### cruise buttons ###
    # TODO: find cruise buttons msg

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends
