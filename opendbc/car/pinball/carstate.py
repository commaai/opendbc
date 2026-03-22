from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.pinball.values import DBC


class CarState(CarStateBase):
  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    ret = structs.CarState()

    ret.gearShifter = structs.CarState.GearShifter.drive
    ret.cruiseState.enabled = True
    ret.cruiseState.available = True

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], [], 0)}
