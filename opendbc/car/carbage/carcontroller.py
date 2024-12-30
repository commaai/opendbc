from opendbc.car.common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # TODO

    new_actuators = actuators.as_builder()

    self.frame += 1
    return new_actuators, can_sends
