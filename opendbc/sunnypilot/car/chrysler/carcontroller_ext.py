from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chrysler.values import RAM_DT
from opendbc.sunnypilot.car.chrysler.values import ChryslerFlagsSP

GearShifter = structs.CarState.GearShifter


class CarControllerExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

  def get_lkas_control_bit(self, CS: CarStateBase, CC: structs.CarControl, lkas_control_bit: bool) -> bool:
    if self.CP_SP.flags & ChryslerFlagsSP.NO_MIN_STEERING_SPEED:
      lkas_control_bit = CC.latActive
    elif self.CP.carFingerprint in RAM_DT:
      if self.CP.minEnableSpeed <= CS.out.vEgo <= self.CP.minEnableSpeed + 0.5:
        lkas_control_bit = True
      if self.CP.minEnableSpeed >= 14.5 and CS.out.gearShifter != GearShifter.drive:
        lkas_control_bit = False

    return lkas_control_bit
