import math
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.cam_lka.bydcan import pack_lkas_hud_status_passthrough
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CANBUS, HUD_MULTIPLIER, CAR, BYD_OP_LONG_PLATFORMS

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["DRIVE_STATE"]["GEAR"]

    self.is_cruise_latch = False
    self.prev_angle = 0
    self.lss_state = 0
    self.lss_alert = 0
    self.tsr = 0
    self.ahb = 0
    self.passthrough = 0
    self.lka_on = 0
    self.HMA = 0
    self.pt2 = 0
    self.pt3 = 0
    self.pt4 = 0
    self.pt5 = 0
    self.lkas_hud_status_passthrough = 0
    self.lkas_rdy_btn = False
    self.op_long = True
    self.distance_val = 1

  def _select_long_parser(self, cp, cp_cam):
    if self.CP.carFingerprint in BYD_OP_LONG_PLATFORMS:
      self.op_long = True
      return cp_cam
    self.op_long = False
    return cp

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    ret.buttonEvents = []

    self.tsr = cp_cam.vl["LKAS_HUD_ADAS"]["TSR"]
    self.lka_on = cp_cam.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_ACTIVE_LOW"]

    self.lkas_rdy_btn = cp.vl["PCM_BUTTONS"]["LKAS_ON_BTN"]
    self.res_btn = cp.vl["PCM_BUTTONS"]["RES_BTN"]
    self.abh = cp_cam.vl["LKAS_HUD_ADAS"]["SET_ME_XFF"]
    self.passthrough = cp_cam.vl["LKAS_HUD_ADAS"]["TSR_STATUS"]
    self.HMA = cp_cam.vl["LKAS_HUD_ADAS"]["HMA"]
    self.pt2 = cp_cam.vl["LKAS_HUD_ADAS"]["PT2"]
    self.pt3 = cp_cam.vl["LKAS_HUD_ADAS"]["PT3"]
    self.pt4 = cp_cam.vl["LKAS_HUD_ADAS"]["PT4"]
    self.pt5 = cp_cam.vl["LKAS_HUD_ADAS"]["PT5"]
    self.lkas_hud_status_passthrough = pack_lkas_hud_status_passthrough(self.abh, self.passthrough)
    self.lkas_healthy = bool(cp_cam.vl["STEERING_MODULE_ADAS"]["EPS_OK"])
    # NOTE: this fork's car.capnp has no lkaDisabled field (kommuai-only schema
    # extension; unused by this project's ALC/desire code either way) — self.lka_on
    # is tracked for when ATTO3's "cam reports LKA off" signal gets a consumer.

    ret.brakeHoldActive = False

    parser_alt = self._select_long_parser(cp, cp_cam)

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_BR"],
    )
    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER
    ret.standstill = ret.vEgoRaw < 0.01

    can_gear = int(cp.vl["DRIVE_STATE"]["GEAR"])

    ret.doorOpen = any([
      cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
      cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
    ])
    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"] == 0

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    if ret.doorOpen or ret.seatbeltUnlatched or ret.brakeHoldActive:
      self.is_cruise_latch = False

    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] >= 0.01

    ret.deprecated.brake = cp.vl["PEDAL"]["BRAKE_PEDAL"]
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"]) or ret.deprecated.brake > 0.01

    ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"]
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"] * steer_dir
    ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 6)

    ret.stockAeb = False
    ret.stockFcw = False
    hud_acc_on1 = bool(parser_alt.vl["ACC_HUD_ADAS"]["ACC_ON1"])
    hud_acc_ctrl = bool(parser_alt.vl["ACC_HUD_ADAS"]["ACC_CONTROLLABLE_AND_ON"])
    ret.cruiseState.available = hud_acc_on1 or hud_acc_ctrl
    if self.CP.carFingerprint == CAR.BYD_SEAL6:
      ret.cruiseState.available |= bool(parser_alt.vl["ACC_HUD_ADAS"]["ACC_ON2"])

    prev_distance_val = self.distance_val
    self.distance_val = int(parser_alt.vl["ACC_HUD_ADAS"]["SET_DISTANCE"]) if self.op_long else 1
    if self.distance_val != prev_distance_val:
      ret.buttonEvents = create_button_events(1, 0, {1: ButtonType.gapAdjustCruise}) + \
                         create_button_events(0, 1, {1: ButtonType.gapAdjustCruise})

    if (cp.vl["PCM_BUTTONS"]["SET_BTN"] != 0 or cp.vl["PCM_BUTTONS"]["RES_BTN"] != 0) and not ret.brakePressed:
      self.is_cruise_latch = True

    if self.op_long and bool(parser_alt.vl["ACC_CMD"]["ACC_REQ_NOT_STANDSTILL"]):
      self.is_cruise_latch = True

    if ret.cruiseState.available:
      ret.cruiseState.speedCluster = max(int(parser_alt.vl["ACC_HUD_ADAS"]["SET_SPEED"]), 30) * CV.KPH_TO_MS
    else:
      ret.cruiseState.speedCluster = 0

    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    # Latch clear from ACC_CMD ACC_CONTROLLABLE_AND_ON: bit often goes low under gas or at standstill
    # while cruise should stay latched; only honor it when moving and not on the accelerator.
    acc_cmd_controllable = self.op_long and bool(parser_alt.vl["ACC_CMD"]["ACC_CONTROLLABLE_AND_ON"])
    acc_cmd_clears_latch = (
      not acc_cmd_controllable and not ret.gasPressed and not ret.standstill
    )
    # Latch clear from HUD ACC_ON1 / ACC_CONTROLLABLE_AND_ON: brief falling edges at standstill while
    # latched; do not drop the latch on HUD loss alone in that case. Include a low-speed band so
    # standstill→rollout does not clear before HUD catches up (standstill often drops first).
    hud_low_speed_hold = ret.standstill or (ret.vEgoRaw < 0.15)
    hud_clears_latch = (
      not ret.cruiseState.available and not (hud_low_speed_hold and self.is_cruise_latch)
    )
    if hud_clears_latch or ret.brakePressed or acc_cmd_clears_latch:
      self.is_cruise_latch = False

    if self.CP.carFingerprint in (CAR.BYD_SEAL, CAR.BYD_SEALION7, CAR.BYD_SEAL6, CAR.BYD_SHARK, CAR.BYD_M6):
      cruise_state = parser_alt.vl["ACC_HUD_ADAS"]["CRUISE_STATE"]
      ret.cruiseState.enabled = cruise_state in (3, 5, 6, 7)
    else:
      ret.cruiseState.enabled = self.is_cruise_latch

    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])
    ret.genericToggle = bool(cp.vl["STALKS"]["GENERIC_TOGGLE"])
    ret.espDisabled = False

    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSM"]["LEFT_APPROACH"])
      ret.rightBlindspot = bool(cp.vl["BSM"]["RIGHT_APPROACH"])

    self.lss_state = cp_cam.vl["LKAS_HUD_ADAS"]["LSS_STATE"]
    self.lss_alert = cp_cam.vl["LKAS_HUD_ADAS"]["SETTINGS"]
    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CarState.get_can_parser(CP),
      Bus.cam: CarState.get_cam_can_parser(CP),
    }

  @staticmethod
  def get_can_parser(CP):
    signals = [
      ("DRIVE_STATE", 50),
      ("PEDAL", 50),
      ("METER_CLUSTER", 20),
      ("STEER_MODULE_2", 100),
      ("STEERING_TORQUE", 50),
      ("STALKS", math.nan),
      ("PCM_BUTTONS", math.nan),
      ("WHEEL_SPEED", 50),
    ]

    if CP.enableBsm:
      signals.append(("BSM", 20))

    if CP.carFingerprint in (CAR.BYD_SEAL, CAR.BYD_SEALION7, CAR.BYD_SHARK):
      signals.append(("ACC_CMD", 50))
      signals.append(("ACC_HUD_ADAS", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("LKAS_HUD_ADAS", 50),
      ("STEERING_MODULE_ADAS", 50),
    ]

    if CP.carFingerprint in BYD_OP_LONG_PLATFORMS:
      signals.append(("ACC_CMD", 50))
      signals.append(("ACC_HUD_ADAS", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.cam_bus)
