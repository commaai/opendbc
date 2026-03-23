from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.pinball import pinballcan
from opendbc.car.interfaces import CarControllerBase
from openpilot.common.swaglog import cloudlog


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])

  def update(self, CC, CS, now_nanos):
    left = 0
    right = 0
    start = 0

    if CC.enabled:
      # A key → negative torque → left flipper
      left = int(CC.actuators.torque < 0)
      # D key → positive torque → right flipper
      right = int(CC.actuators.torque > 0)
      # W key → negative accel → start button
      start = int(CC.actuators.accel < 0)

    can_sends = [pinballcan.create_solenoid_cmd(self.packer, left, right, start)]

    new_actuators = CC.actuators.as_builder()
    self.frame += 1
    return new_actuators, can_sends
