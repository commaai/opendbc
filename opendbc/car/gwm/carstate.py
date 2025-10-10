from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import DBC

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    ret = structs.CarState()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["FRONT_RIGHT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_RIGHT_WHEEL_SPEED"]
    )

    ret.standstill = cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"] < 1
    ret.vEgoRaw = cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"]
    ret.brakePressed = cp.vl["BRAKE"]["BRAKE_PRESSURE"] > 1
    ret.gearShifter = GearShifter.drive

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
