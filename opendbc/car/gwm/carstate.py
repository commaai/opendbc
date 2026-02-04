from opendbc.car import structs, Bus, CanBusBase
from opendbc.can.parser import CANParser
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import DBC
# DEBUG
from openpilot.common.params import Params
# DEBUG
import copy

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # Store original message to copy it later in carcontroller
    self.steer_and_ap_stalk_msg = {}
    self.eps_stock_values = {}
    self.camera_stock_values = {}

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    ret = structs.CarState()

    # Store the original message to reuse in carcontroller
    self.steer_and_ap_stalk_msg = copy.copy(cp.vl["STEER_AND_AP_STALK"])
    self.eps_stock_values = copy.copy(cp.vl["RX_STEER_RELATED"])
    self.camera_stock_values = copy.copy(cp.vl["STEER_CMD"])

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["FRONT_RIGHT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_RIGHT_WHEEL_SPEED"]
    )

    ret.standstill = abs(ret.vEgoRaw) < 1e-3
    ret.gasPressed = cp.vl["CAR_OVERALL_SIGNALS2"]["GAS_POSITION"] > 0
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]
    ret.brakePressed = cp.vl["BRAKE"]["BRAKE_PRESSURE"] > 0
    ret.parkingBrake = cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"] == 0

    ret.gearShifter = GearShifter.drive if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 1 else \
                      GearShifter.neutral if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 2 else \
                      GearShifter.reverse if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 3 else \
                      GearShifter.park

    ret.steeringAngleDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_ANGLE"] * (-1 if cp.vl["STEER_AND_AP_STALK"]["STEERING_DIRECTION"] else 1)
    ret.steeringRateDeg = 0 # TODO
    ret.steeringTorque = cp.vl["RX_STEER_RELATED"]["B_RX_DRIVER_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > 50

    ret.doorOpen = any([cp.vl["DOOR_DRIVER"]["DOOR_REAR_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_FRONT_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_REAR_LEFT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_DRIVER_OPEN"]])
    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT"]["SEAT_BELT_DRIVER_STATE"])
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"],
                                                                      cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"])
    # DEBUG
    ret.cruiseState.available = ret.cruiseState.enabled = Params().get_bool("AleSato_DebugButton1")
    # DEBUG
    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Compute bus offset from number of safetyConfigs so multipanda setups
    # (internal + external pandas) map DBCs to the correct physical bus.
    can_base = CanBusBase(CP, None)
    main_bus = can_base.offset
    adas_bus = can_base.offset + 1
    cam_bus = can_base.offset + 2

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], main_bus),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], adas_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], cam_bus),
    }
