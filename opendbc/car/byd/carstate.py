from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CANBUS, BydSteerStates

# Multiplier between GPS ground speed to the meter cluster's displayed speed
HUD_MULTIPLIER = 1.068

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.shifter_values = can_define.dv["DRIVE_STATE"]['GEAR']
    self.set_distance_values = can_define.dv['ACC_HUD_ADAS']['SET_DISTANCE']

    self.prev_angle = 0
    self.hud_passthrough = 0
    self.adas_settings_pt = 0
    self.steer_state = 0
    self.lka_on = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    self.adas_settings_pt = cp_cam.vl["LKAS_HUD_ADAS"]['ADAS_SETTINGS_PT']
    self.hud_passthrough = cp_cam.vl["LKAS_HUD_ADAS"]['HUD_PASSTHROUGH']
    self.lka_on = cp_cam.vl["LKAS_HUD_ADAS"]['LKAS_ENABLED_ACTIVE_LOW']

    self.steer_state = cp_cam.vl["STEERING_MODULE_ADAS"]['STEER_STATE']
    ret.steerFaultTemporary = self.steer_state == BydSteerStates.FAULTED

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FR'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_BL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_BR'],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.rl + ret.wheelSpeeds.fl) / 2.0

    # unfiltered speed from CAN sensors
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER
    ret.standstill = ret.vEgoRaw < 0.01

    # safety checks to engage
    can_gear = int(cp.vl["DRIVE_STATE"]['GEAR'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['BACK_LEFT_DOOR'],
                       cp.vl["METER_CLUSTER"]['FRONT_LEFT_DOOR'],
                       cp.vl["METER_CLUSTER"]['BACK_RIGHT_DOOR'],
                       cp.vl["METER_CLUSTER"]['FRONT_RIGHT_DOOR']])

    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEATBELT_DRIVER'] == 0

    # gas pedal
    ret.gas = cp.vl["PEDAL"]['GAS_PEDAL']
    ret.gasPressed = ret.gas >= 0.01

    # brake pedal
    ret.brake = cp.vl["PEDAL"]['BRAKE_PEDAL']
    ret.brakePressed = ret.brake > 0.01

    # steer
    ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]['STEER_ANGLE_2']
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE']
    ret.steeringTorqueEps = cp.vl["STEER_MODULE_2"]['DRIVER_EPS_TORQUE']
    ret.steeringPressed = bool(ret.steeringTorqueEps > 6)

    # TODO: get the real value
    ret.stockAeb = False
    ret.stockFcw = False
    ret.cruiseState.available = bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON1"]) or bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON2"])

    # byd speedCluster will follow wheelspeed if cruiseState is not available
    if ret.cruiseState.available:
      ret.cruiseState.speedCluster = max(int(cp_cam.vl["ACC_HUD_ADAS"]['SET_SPEED']), 30) * CV.KPH_TO_MS
    else:
      ret.cruiseState.speedCluster = 0

    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = bool(cp_cam.vl["ACC_CMD"]["STANDSTILL_STATE"])
    ret.cruiseState.nonAdaptive = False

    ret.cruiseState.enabled = not bool(cp_cam.vl["ACC_CMD"]["CMD_REQ_ACTIVE_LOW"])

    # button presses
    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])
    ret.genericToggle = bool(cp.vl["STALKS"]["GENERIC_TOGGLE"])
    ret.espDisabled = False

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      ret.leftBlindspot = bool(cp.vl["BSM"]["LEFT_APPROACH"])
      ret.rightBlindspot = bool(cp.vl["BSM"]["RIGHT_APPROACH"])

    return ret


  @staticmethod
  def get_can_parsers(CP):
    pt_signals = [
      ("DRIVE_STATE", 50),
      ("WHEEL_SPEED", 50),
      ("PEDAL", 50),
      ("METER_CLUSTER", 20),
      ("STEER_MODULE_2", 100),
      ("STEERING_TORQUE", 50),
      ("STALKS", 20),
      ("BSM", 20),
      ("PCM_BUTTONS", 20),
    ]

    cam_signals = [
      ("ACC_HUD_ADAS", 50),
      ("ACC_CMD", 50),
      ("LKAS_HUD_ADAS", 50),
      ("STEERING_MODULE_ADAS", 50),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_signals, CANBUS.main_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_signals, CANBUS.cam_bus),
    }
