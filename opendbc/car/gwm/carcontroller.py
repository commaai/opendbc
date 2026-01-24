from opendbc.can.packer import CANPacker
from opendbc.car import Bus, CanBusBase
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm.gwmcan import create_helloworld


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.status = 2

    # Compute panda multipanda offset based on safetyConfigs length. This let's us
    # add the offset to all bus numbers when an external panda is present.
    can_base = CanBusBase(self.CP, None)
    self.bus_main = can_base.offset

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    can_sends.append(create_helloworld(self.packer, bus=self.bus_main))
    new_actuators = actuators.as_builder()
    self.frame += 1
    return new_actuators, can_sends
