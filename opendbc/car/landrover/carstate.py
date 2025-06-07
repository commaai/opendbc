import math
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.landrover.values import CAR, DBC, CanBus, CarControllerParams

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    if CP.carFingerprint in (CAR.LANDROVER_DEFENDER_2023):
      self.shifter_values = self.can_define.dv["GearPRND"]["PRND"]

    self.is_metric = True
    self.params = CarControllerParams(CP)
    self.wheelbase = CP.wheelbase


  def update(self, can_parsers) -> structs.CarState:
    if self.CP.carFingerprint in (CAR.LANDROVER_DEFENDER_2023):
      return self.update_can_defender(can_parsers)

  def update_can_defender(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()

    self.is_metric = True

    ret.seatbeltUnlatched = (cp.vl["SeatBelt"]["SeatBelt_Driver"]  == 0)
    ret.doorOpen = not any([cp.vl["DoorStatus"]["FrontLeftDoor"], \
         cp.vl["DoorStatus"]["FrontRightDoor"], \
         cp.vl["DoorStatus"]["RearLeftDoor"], \
         cp.vl["DoorStatus"]["RearRightDoor"]])


    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WheelSpeedFront"]["SpeedLeft"],
      cp.vl["WheelSpeedFront"]["SpeedRight"],
      cp.vl["WheelSpeedRear"]["SpeedLeft"],
      cp.vl["WheelSpeedRear"]["SpeedRight"],
    )

    #Speed = cp.vl["Info02"]["WheelSpeed"]
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
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

    ret.gas = cp.vl["GasPedal"]["GasPedalPos"]
    ret.gasPressed = cp.vl["GasPedal_ON"]["GasPedalDriver"] == 1


    if self.CP.enableBsm:
      ret.leftBlindspot  = cp.vl["BlindSpot"]["LeftBS"] == 4
      ret.rightBlindspot = cp.vl["BlindSpot"]["RightBS"] == 4

    ret.stockAeb = False

    ret.cruiseState.available = cp.vl["CruiseInfo"]["CruiseOn"] == 1
    ret.cruiseState.enabled =  cp.vl["CruiseInfo"]["CruiseOn"] == 1
    ret.cruiseState.speed = ret.vEgoRaw
    ret.cruiseState.standstill = False


    return ret

  def get_can_parsers(self, CP):
     return self.get_can_parser_defender(CP)


  def get_can_parser_defender(self, CP):
    pt_messages = [
      # sig_name, freq
      ("SWM_Angle", 100),
      ("SWM_Torque", 50),
      ("PSCM_Out", 50),
      ("WheelSpeedFront", 25),
      ("WheelSpeedRear", 25),
      ("BlindSpot", 6),
      ("GearPRND", 10),
      ("HandleSignal", 100),
      ("SeatBelt", 100),
      ("LKAS_BTN", 16),
      ("CruiseInfo", 25),
      ("StopAndGo", 50),
      ("GasPedal", 50),
      ("GasPedal_ON", 10),
      ("Info02", 25),
      ("DoorStatus", 4),
    ]

    """
    c2f_messages = [
      ("LKAS_OP_TO_FLEXRAY", 50),
    ]
    """

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.UNDERBODY),

      #Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], c2f_messages, CanBus.CAN2FLEXRAY)

    }

