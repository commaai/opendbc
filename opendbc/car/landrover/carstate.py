import math
#from opendbc.car.common.conversions import Conversions as CV
from opendbc.can import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.interfaces import CarStateBase
from opendbc.car.landrover.values import DBC, CanBus, CarControllerParams, LandroverFlags

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    if CP.flags & LandroverFlags.FLEXRAY_HARNESS:
      self.shifter_values = can_define.dv["GearPRND"]["PRND"]
    else:
      self.shifter_values = can_define.dv["GEAR_PRND"]["GEAR_SHIFT"]

    self.is_metric = True
    self.params = CarControllerParams(CP)
    self.wheelbase = CP.wheelbase

    self.lc_button = 0

  def update(self, can_parsers) -> structs.CarState:
    if self.CP.flags & LandroverFlags.FLEXRAY_HARNESS:
      return self.update_can_defender(can_parsers)

    cp = can_parsers[Bus.pt]

    ret = structs.CarState()

    self.is_metric = True

    ret.seatbeltUnlatched = (cp.vl["SEAT_BELT"]["SEAT_BELT_DRIVER"]  == 0)
    ret.seatbeltUnlatched = False
    ret.doorOpen = False

    self.parse_wheel_speeds(ret,
      cp.vl["SPEED_01"]["SPEED01"],
      cp.vl["SPEED_02"]["SPEED02"],
      cp.vl["SPEED_01"]["SPEED01"],
      cp.vl["SPEED_02"]["SPEED02"],
    )

    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = cp.vl["EPS_01"]["STEER_ANGLE01"]
    ret.steeringRateDeg = cp.vl["EPS_01"]["STEER_SPEED01"]

    steer_rad = math.radians(ret.steeringAngleDeg)
    yawrate_rad_s = math.tan(steer_rad) * ret.vEgo / self.wheelbase

    ret.yawRate = math.degrees(yawrate_rad_s)

    # TODO torq TorqEPS Pressed
    ret.steeringTorque = cp.vl["EPS_00"]["STEER_RATE00"]
    ret.steeringTorqueEps = cp.vl["EPS_02"]["STEER_TORQUE_DRIVER02"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["TURN_SIGNAL"]['LEFT_BLINK'],cp.vl["TURN_SIGNAL"]['RIGHT_BLINK'])

    gear = cp.vl["GEAR_PRND"]["GEAR_SHIFT"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    ret.brake = 0
    ret.brakePressed = (cp.vl["CRUISE_CONTROL"]['DRIVER_BRAKE'] == 1)

    gas = (cp.vl["ACCELATOR_DRIVER"]["ACCELATOR_DRIVER"])
    ret.gasPressed = gas >= 1

    if self.CP.enableBsm:
      ret.leftBlindspot  = cp.vl["LEFT_ALERT"]["LEFT_ALERT_1"] != 0
      ret.rightBlindspot = cp.vl["RIGHT_ALERT"]["RIGHT_ALERT_1"] != 0
    else:
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    ret.stockAeb = False

    ret.blockPcmEnable = False

    ret.cruiseState.available = cp.vl["CRUISE_CONTROL"]["CRUISE_ON"] == 1
    ret.cruiseState.enabled = cp.vl["CRUISE_CONTROL"]["CRUISE_ON"] == 1
    ret.cruiseState.speed = ret.vEgoRaw
    ret.cruiseState.standstill = False

    return ret

  def update_can_defender(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()

    self.is_metric = True
    #speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    ret.seatbeltUnlatched = (cp.vl["SeatBelt"]["SeatBelt_Driver"]  == 0)
    ret.doorOpen = not any([cp.vl["DoorStatus"]["FrontLeftDoor"],
         cp.vl["DoorStatus"]["FrontRightDoor"],
         cp.vl["DoorStatus"]["RearLeftDoor"],
         cp.vl["DoorStatus"]["RearRightDoor"]])

    self.parse_wheel_speeds(ret,
      cp.vl["WheelSpeedFront"]["SpeedLeft"],
      cp.vl["WheelSpeedFront"]["SpeedRight"],
      cp.vl["WheelSpeedRear"]["SpeedLeft"],
      cp.vl["WheelSpeedRear"]["SpeedRight"],
    )

    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = cp.vl["SWM_Angle"]["SteerAngle"]
    ret.steeringRateDeg = cp.vl["SWM_Angle"]["SteerRate"]  # TODO

    steer_rad = math.radians(ret.steeringAngleDeg)
    yawrate_rad_s = math.tan(steer_rad) * ret.vEgo / self.wheelbase

    ret.yawRate = math.degrees(yawrate_rad_s)

    # TODO torq TorqEPS Pressed
    ret.steeringTorque = cp.vl["SWM_Torque"]["TorqueDriver"]
    ret.steeringTorqueEps = cp.vl["PSCM_Out"]["AngleTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["HandleSignal"]['TurnLeft'],cp.vl["HandleSignal"]['TurnRight'])

    gear = cp.vl["GearPRND"]["PRND"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    ret.brake = cp.vl["Info02"]["BrakePedalPos"]
    ret.brakePressed = cp.vl["StopAndGo"]["BrakeDriver"] == 1
    ret.gasPressed = cp.vl["GasPedal_ON"]["GasPedalDriver"] == 1

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BlindSpot"]["LeftBS"] == 4
      ret.rightBlindspot = cp.vl["BlindSpot"]["RightBS"] == 4

    ret.stockAeb = False

    ret.cruiseState.enabled = cp.vl["CruiseInfo"]["CruiseOn"] == 1
    ret.cruiseState.speed = ret.vEgoRaw
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = False

    prev_lc_button = self.lc_button
    self.lc_button = bool(cp.vl["LKAS_BTN"]["LKAS_Btn_on"])
    ret.cruiseState.available = self.lc_button

    ret.buttonEvents = [
      *create_button_events(self.lc_button, prev_lc_button, {1: ButtonType.lkas}),
    ]

    return ret

  def get_can_parser_defender(self, CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.UNDERBODY),
    }

  def get_can_parsers(self, CP):
    if CP.flags & LandroverFlags.FLEXRAY_HARNESS:
      return self.get_can_parser_defender(CP)

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.UNDERBODY),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.CAM)
    }
