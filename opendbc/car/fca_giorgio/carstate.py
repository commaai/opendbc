from opendbc.car.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.fca_giorgio.values import DBC, CanBus, CarControllerParams


GearShifter = structs.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.frame = 0
    self.CCP = CarControllerParams(CP)
    self.lka_status = 0

  def update(self, can_parsers) -> structs.CarState:
    pt_cp = can_parsers[Bus.pt]
    pt_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_FL"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_FR"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_RL"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_RR"],
      unit=1,
    )
    ret.standstill = ret.vEgoRaw == 0

    ret.steeringAngleDeg = pt_cp.vl["EPS_1"]["STEERING_ANGLE"]
    ret.steeringRateDeg = pt_cp.vl["EPS_1"]["STEERING_RATE"]
    ret.steeringTorque = pt_cp.vl["EPS_2"]["DRIVER_TORQUE"]
    # ret.steeringTorqueEps = pt_cp.vl["EPS_3"]["EPS_TORQUE"]
    ret.steeringPressed = ret.steeringTorque > 80
    ret.yawRate = pt_cp.vl["ABS_2"]["YAW_RATE"]
    ret.steerFaultPermanent = bool(pt_cp.vl["EPS_2"]["LKA_FAULT"])
    self.lka_status = pt_cp.vl["EPS_2"]["LKA_STATUS"]

    ret.gasPressed = pt_cp.vl["ENGINE_1"]["ACCEL_PEDAL"] > 0
    ret.brakePressed = bool(pt_cp.vl["ABS_3"]["BRAKE_PEDAL_SWITCH"])
    #ret.parkingBrake = TODO

    if bool(pt_cp.vl["ENGINE_1"]["REVERSE"]):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # TODO: is it ok that speed is gated by "enabled"?
    # TODO: correct units on speed?
    # NOTE: CRUISE_MODE glitches to zero for a frame on ACC enable
    ret.cruiseState.enabled = bool(pt_cam.vl["ACC_2"]["CRUISE_STATUS"])
    ret.cruiseState.available = pt_cam.vl["ACC_4"]["CRUISE_MODE"] == 2 or ret.cruiseState.enabled
    ret.cruiseState.speed = pt_cam.vl["ACC_2"]["HUD_SPEED"] * CV.KPH_TO_MS

    ret.leftBlinker = bool(pt_cp.vl["BCM_1"]["LEFT_TURN_STALK"])
    ret.rightBlinker = bool(pt_cp.vl["BCM_1"]["RIGHT_TURN_STALK"])
    # ret.buttonEvents = TODO
    # ret.espDisabled = TODO

    # ret.leftBlindspot = bool(pt_cp.vl["BLIND_SPOT"]["BLIND_SPOT_LEFT"])
    # ret.rightBlindspot = bool(pt_cp.vl["BLIND_SPOT"]["BLIND_SPOT_RIGHT"])

    # ret.doorOpen = bool(pt_cp.vl["BCM_2"]["DOOR_OPEN_FL"])
    ret.seatbeltUnlatched = bool(pt_cp.vl["BCM_2"]["SEATBELT_UNBUCKLED_FL"])

    self.frame += 1
    return ret

  @staticmethod
  def get_can_parsers(CP):

    # manually configure some optional and variable-rate/edge-triggered messages
    pt_messages, cam_messages = [], []

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).cam),
    }
