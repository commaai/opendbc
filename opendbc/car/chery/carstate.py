from opendbc.can import CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chery.values import (
  CAM_PARSER_MSGS,
  CANBUS,
  CAR,
  DBC,
  GEAR_MAP,
  ICAUR_BLINKER_LEFT,
  ICAUR_BLINKER_RIGHT,
  ICAUR_BRAKE_PRESSED,
  ICAUR_CAM_PARSER_MSGS,
  ICAUR_GAS_PRESSED,
  ICAUR_GEAR_MAP,
  ICAUR_PT_PARSER_MSGS,
  ICAUR_STEERING_PRESSED_MIN_COUNT,
  ICAUR_STEERING_PRESSED_TORQUE,
  OMODA_BRAKE_PRESSURE_RAW_MAX,
  OMODA_BRAKE_PRESSURE_RAW_MIN,
  OMODA_CAM_PARSER_MSGS,
  OMODA_GEAR_MAP,
  OMODA_PT_PARSER_MSGS,
  PT_PARSER_MSGS,
  STEER_RELATED_INTERVENTION_DEG_MIN,
)

ButtonType = structs.CarState.ButtonEvent.Type

# Camera HUD fields used for cruise/ADAS state (and Jaecoo HUD override TX when enabled).
_CAM_HUD_FIELDS = (
  "AEB", "CANCEL_CRUISE_UNCERTAIN", "GAS_RESUME_UNCERTAIN", "FOLLOW_DISTANCE",
  "NEW_SIGNAL_1", "PCW", "CRUISE_STATE", "GAS_OVERRIDE", "AEB_RELATED", "SET_SPEED",
  "HANDS_ON_WHEEL_STEER",
)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.prev_icc = False
    self.prev_cruise = False
    self.prev_angle = 0.0
    # Exposed to CarController:
    self.pcm_button_counter = 0
    self.lkas_info_steer_related = 0.0
    self.steer_related_intervention = False
    self.cam_hud = {f: 0 for f in _CAM_HUD_FIELDS}
    # Live EPS snapshot — used by CarController to rebuild EPS on bus 2 byte-identical
    # to stock (panda blocks native fwd while our spoof loop is active).
    self.eps_steering_angle = 0.0
    self.eps_driver_torque = 0
    self.eps_counter = 0
    self.cruise_state = 1  # HUD CRUISE_STATE: 3=ENABLE 2=READY 1=IDLE

  def update(self, can_parsers):
    cp, cam = can_parsers[Bus.pt], can_parsers[Bus.cam]
    ret = structs.CarState()
    omoda = self.CP.carFingerprint == CAR.CHERY_OMODA_5
    icaur = self.CP.carFingerprint == CAR.CHERY_ICAUR_03

    # --- Wheels / pedals / gear ---
    if icaur:
      ws_a = cp.vl["ICAUR_WHEELSPEED_A"]
      ws_b = cp.vl["ICAUR_WHEELSPEED_B"]
      # DBC signals are already m/s — do not apply KPH_TO_MS.
      self.parse_wheel_speeds(
        ret,
        ws_a["WHEEL_FL"], ws_a["WHEEL_FR"],
        ws_b["WHEEL_BL"], ws_b["WHEEL_BR"],
        unit=1.0,
      )
    else:
      self.parse_wheel_speeds(
        ret, cp.vl["WHEELSPEED_2"]["WHEEL_FL"], cp.vl["WHEELSPEED_2"]["WHEEL_FR"],
        cp.vl["WHEELSPEED_1"]["WHEEL_BL"], cp.vl["WHEELSPEED_1"]["WHEEL_BR"],
      )
    ret.vEgoCluster = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.01

    if icaur:
      brake = max(0.0, min(float(cp.vl["ICAUR_BRAKE"]["BRAKE_PRESSURE"]), 1.0))
      gas = max(0.0, min(float(cp.vl["ICAUR_GAS"]["GAS_PEDAL_PRESSURE"]), 1.0))
      # Angle + driver torque both on STEER_RELATED (0xC4).
      steer = cp.vl["STEER_RELATED"]
      ret.steeringAngleDeg = max(-450.0, min(450.0, float(steer["STEERING_ANGLE"])))
      ret.steeringTorque = float(steer["DRIVER_TORQUE"])
      # DRIVER_TORQUE is unsigned; infer sign from angle delta for steeringTorqueEps.
      steer_dir = 1 if ret.steeringAngleDeg >= self.prev_angle else -1
      self.prev_angle = ret.steeringAngleDeg
      ret.steeringTorqueEps = ret.steeringTorque * steer_dir
      ret.steeringPressed = self.update_steering_pressed(
        abs(ret.steeringTorque) >= ICAUR_STEERING_PRESSED_TORQUE,
        ICAUR_STEERING_PRESSED_MIN_COUNT,
      )
      ret.brakePressed = brake >= ICAUR_BRAKE_PRESSED
      ret.deprecated.brake = brake if ret.brakePressed else 0.0
      ret.gasPressed = gas >= ICAUR_GAS_PRESSED
      # CarState.gas was removed (gasDEPRECATED); only gasPressed is published.
      ret.gearShifter = self.parse_gear_shifter(
        ICAUR_GEAR_MAP.get(int(cp.vl["ICAUR_TRANSMISSION"]["GEAR"]))
      )
      self.eps_steering_angle = ret.steeringAngleDeg
      self.eps_driver_torque = int(steer["DRIVER_TORQUE"])
      self.eps_counter = int(steer["COUNTER"])
    else:
      eps = cp.vl["EPS"]
      ret.steeringAngleDeg = float(eps["STEERING_ANGLE"])
      ret.steeringTorque = eps["DRIVER_TORQUE"]
      self.eps_steering_angle = float(eps["STEERING_ANGLE"])
      self.eps_driver_torque = int(eps["DRIVER_TORQUE"])
      self.eps_counter = int(eps["COUNTER"])
      # DRIVER_TORQUE is unsigned; infer sign from angle delta for steeringTorqueEps.
      steer_dir = 1 if ret.steeringAngleDeg >= self.prev_angle else -1
      self.prev_angle = ret.steeringAngleDeg
      ret.steeringTorqueEps = ret.steeringTorque * steer_dir
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 5, 5)

      ret.gasPressed = cp.vl["GAS"]["GAS_PEDAL_PRESSURE"] > 0.01
    if omoda:
      raw = int(cp.vl["OMODA_BRAKE"]["BRAKE_PRESSURE"] / 0.0188679)
      if raw <= OMODA_BRAKE_PRESSURE_RAW_MAX:
        ret.deprecated.brake = raw / OMODA_BRAKE_PRESSURE_RAW_MAX
        ret.brakePressed = raw > OMODA_BRAKE_PRESSURE_RAW_MIN
      else:
        ret.deprecated.brake = 0.0
        ret.brakePressed = False
      ret.gearShifter = self.parse_gear_shifter(OMODA_GEAR_MAP.get(int(cp.vl["OMODA_TRANSMISSION"]["GEAR"])))
    elif not icaur:
      ret.deprecated.brake = cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSURE"]
      ret.brakePressed = ret.deprecated.brake > 0.01
      ret.gearShifter = self.parse_gear_shifter(GEAR_MAP.get(int(cp.vl["TRANSMISSION"]["GEAR"])))

    # --- Body / stalk ---
    if icaur:
      blink = int(cp.vl["ICAUR_STALK"]["BLINKER"])
      ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
        3, blink in ICAUR_BLINKER_LEFT, blink in ICAUR_BLINKER_RIGHT,
      )
      # Door / seatbelt / generic toggle not reverse-engineered yet.
      ret.doorOpen = False
      ret.genericToggle = False
      ret.seatbeltUnlatched = False
    else:
      stalk = cp.vl["STALK"]
      ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
        3, stalk["LEFT_BLINKER"], stalk["RIGHT_BLINKER"],
      )
      ret.doorOpen = bool(int(stalk["PAYLOAD391_B3"]) & 1)
      ret.genericToggle = bool(stalk["GENERIC_TOGGLE"])
      ret.seatbeltUnlatched = bool(
        cp.vl["SEATBELT_287"]["DRIVER_UNBUCKLED"] or cp.vl["SEATBELT_430"]["DRIVER_UNBUCKLED"]
      )
    ret.espDisabled = False

    # --- Cruise / HUD ---
    hud = cp.vl["HUD"] if omoda else cam.vl["HUD"]
    self.cam_hud = {f: hud[f] for f in _CAM_HUD_FIELDS}
    ret.stockAeb = bool(hud["AEB"])
    ret.stockFcw = bool(hud["PCW"])

    set_kph = float(hud["SET_SPEED"])
    self.cruise_state = int(hud["CRUISE_STATE"])
    ret.cruiseState.available = True
    ret.cruiseState.enabled = self.cruise_state == 3
    ret.cruiseState.speed = ret.cruiseState.speedCluster = set_kph * CV.KPH_TO_MS if set_kph > 0 else 0.0
    ret.cruiseState.standstill = ret.standstill and ret.cruiseState.enabled
    ret.cruiseState.nonAdaptive = False

    # --- PCM buttons ---
    if icaur:
      self.pcm_button_counter = 0
      ret.buttonEvents = []
    else:
      pcm = cp.vl["PCM_BUTTONS"]
      self.pcm_button_counter = int(pcm["COUNTER"])
      icc, cruise_btn = bool(pcm["ICC_TOGGLE"]), bool(pcm["CRUISE_BUTTON"])
      ret.buttonEvents = (
        create_button_events(icc, self.prev_icc, {1: ButtonType.altButton2}) +
        create_button_events(cruise_btn, self.prev_cruise, {1: ButtonType.mainCruise})
      )
      self.prev_icc, self.prev_cruise = icc, cruise_btn

    # --- ADAS / LKAS state used by CarController ---
    self.lkas_info_steer_related = 0.0 if icaur else float(cp.vl["LKAS_INFO"]["STEER_RELATED"])
    # Jaecoo: STEER_RELATED angle raw>=36000 is a status code (decoded ≈342.7°), not road angle.
    # iCaur: same bits are real degrees — rely on DRIVER_TORQUE via steeringPressed instead.
    self.steer_related_intervention = (
      False if icaur else
      float(cp.vl["STEER_RELATED"]["STEERING_ANGLE"]) >= STEER_RELATED_INTERVENTION_DEG_MIN
    )

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {Bus.pt: CarState.get_can_parser(CP), Bus.cam: CarState.get_cam_can_parser(CP)}

  @staticmethod
  def get_can_parser(CP):
    if CP.carFingerprint == CAR.CHERY_OMODA_5:
      msgs = OMODA_PT_PARSER_MSGS
    elif CP.carFingerprint == CAR.CHERY_ICAUR_03:
      msgs = ICAUR_PT_PARSER_MSGS
    else:
      msgs = PT_PARSER_MSGS
    return CANParser(DBC[CP.carFingerprint]["pt"], msgs, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.carFingerprint == CAR.CHERY_OMODA_5:
      msgs = OMODA_CAM_PARSER_MSGS
    elif CP.carFingerprint == CAR.CHERY_ICAUR_03:
      msgs = ICAUR_CAM_PARSER_MSGS
    else:
      msgs = CAM_PARSER_MSGS
    return CANParser(DBC[CP.carFingerprint]["pt"], msgs, CANBUS.cam_bus)
