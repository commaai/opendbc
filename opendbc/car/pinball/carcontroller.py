from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.pinball import pinballcan
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])

  def update(self, CC, CS, now_nanos):
    left = 0
    right = 0
    start = 0

    if CC.enabled:
      # Connect's default wideRoad view sends A as positive steering and W/S
      # as positive/negative acceleration. A and S can activate both paddles.
      left = int(CC.actuators.torque > 0.5)    # A: left paddle
      right = int(CC.actuators.accel < -0.5)   # S: right paddle
      start = int(CC.actuators.accel > 0.5)    # W: middle servo (SOLENOID_START)

    can_sends = [pinballcan.create_solenoid_cmd(self.packer, left, right, start)]

    new_actuators = CC.actuators.as_builder()
    self.frame += 1
    return new_actuators, can_sends
