import copy
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.carbage.values import DBC

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

  def update(self, can_parsers) -> structs.CarState:
    cp_cbp = can_parsers[Bus.main]
    cp_ibst = can_parsers[Bus.adas]
    ret = structs.CarState()

    return ret

  @staticmethod
  def get_can_parsers(CP):

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], [], 2)
    }
