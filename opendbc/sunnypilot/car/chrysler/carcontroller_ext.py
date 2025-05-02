from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chrysler.values import RAM_DT

GearShifter = structs.CarState.GearShifter


class CarControllerExt:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP

  def get_lkas_control_bit(self, CS: CarStateBase, lkas_control_bit: bool, lkas_control_bit_prev: bool) -> bool:
    if self.CP.carFingerprint in RAM_DT:
      lkas_control_bit = lkas_control_bit_prev
      if self.CP.minEnableSpeed <= CS.out.vEgo <= self.CP.minEnableSpeed + 0.5:
        lkas_control_bit = True
      if CS.out.gearShifter != GearShifter.drive:
        lkas_control_bit = False

    return lkas_control_bit
