import copy
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, CanSignalRateCalculator, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import CAR, DBC, CANBUS, GEAR_MAP, PLATFORM_3Y, DOORS, STEER_THRESHOLD

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
    self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
    self.can_defines = {
      **self.can_define_party.dv,
      **self.can_define_pt.dv,
      **self.can_define_chassis.dv,
    }

    if self.CP.carFingerprint not in PLATFORM_3Y:
      # TODO: this should be swapped on the harnesses
      CANBUS.chassis = 1
      CANBUS.radar = 5

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False

    self.hands_on_level = 0
    self.das_control = None
    self.angle_rate_calulator = CanSignalRateCalculator(100)

  def update_autopark_state(self, autopark_state: str, cruise_enabled: bool):
    autopark_now = autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if autopark_now and not self.autopark_prev and not self.cruise_enabled_prev:
      self.autopark = True
    if not autopark_now:
      self.autopark = False
    self.autopark_prev = autopark_now
    self.cruise_enabled_prev = cruise_enabled

  def update(self, can_parsers) -> structs.CarState:
    is_3Y = self.CP.carFingerprint in PLATFORM_3Y

    cp_party = can_parsers[Bus.party]
    cp_ap = can_parsers[Bus.ap_party] if is_3Y else can_parsers[Bus.ap_pt]
    cp_pt = can_parsers[Bus.pt]
    cp_chassis = can_parsers[Bus.chassis] if not is_3Y else None
    ret = structs.CarState()

    # Vehicle speed
    if is_3Y:
      ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    else:
      ret.vEgoRaw = cp_party.vl["ESP_private1"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    if is_3Y:
      pedal_status = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"]
    else:
      pedal_status = cp_pt.vl["DI_torque1"]["DI_pedalPos"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = pedal_status > 0

    # Brake pedal
    ret.brake = 0

    if is_3Y:
      ret.brakePressed = cp_party.vl["IBST_status"]["IBST_driverBrakeApply"] == 2
    else:
      ret.brakePressed = cp_party.vl["IBST_private2"]["IBST_brakePedalApplied"] == 1

    # Steering wheel
    epas_name = "EPAS3S" if is_3Y else "EPAS"
    epas_status = cp_party.vl[f"{epas_name}_sysStatus"]
    self.hands_on_level = epas_status[f"{epas_name}_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status[f"{epas_name}_internalSAS"]
    ret.steeringRateDeg = self.angle_rate_calulator.update(ret.steeringAngleDeg, epas_status[f"{epas_name}_sysStatusCounter"])
    ret.steeringTorque = -epas_status[f"{epas_name}_torsionBarTorque"]

    # This matches stock logic, but with halved minimum frames (0.25-0.3s)
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 15)
    eac_status = self.can_defines[f"{epas_name}_sysStatus"][f"{epas_name}_eacStatus"].get(int(epas_status[f"{epas_name}_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    # TODO: implement in safety
    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    di_state = cp_party.vl["DI_state"] if is_3Y else cp_pt.vl["DI_state"]

    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(di_state["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(di_state["DI_speedUnits"]), None)

    autopark_state = self.can_define.dv["DI_state"]["DI_autoparkState"].get(int(cp_party.vl["DI_state"]["DI_autoparkState"]), None)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.update_autopark_state(autopark_state, cruise_enabled)

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled and not self.autopark
    if speed_units == "KPH":
      ret.cruiseState.speed = max(di_state["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(di_state["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    if is_3Y:
      ret.gearShifter = GEAR_MAP[self.can_defines["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]
    else:
      ret.gearShifter = GEAR_MAP[self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_pt.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    if is_3Y:
      ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1
      ret.leftBlinker = cp_party.vl["UI_warning"]["leftBlinkerBlinking"] in (1, 2)
      ret.rightBlinker = cp_party.vl["UI_warning"]["rightBlinkerBlinking"] in (1, 2)
      ret.seatbeltUnlatched = cp_party.vl["UI_warning"]["buckleStatus"] != 1
      ret.leftBlindspot = cp_ap.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
      ret.rightBlindspot = cp_ap.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0
    else:
      ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)
      ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
      ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1
      ret.seatbeltUnlatched = cp_chassis.vl["DriverSeat"]["buckleStatus"] != 1

    # AEB
    ret.stockAeb = cp_ap.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap.vl["DAS_control"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    party_messages = []
    ap_messages = [
      ("DAS_control", 25),
      ("DAS_steeringControl", 50),
      ("DAS_status", 2),
    ]

    if CP.carFingerprint in PLATFORM_3Y:
      party_messages += [
        ("DI_speed", 50),
        ("DI_systemStatus", 100),
        ("IBST_status", 25),
        ("DI_state", 10),
        ("EPAS3S_sysStatus", 100),
        ("UI_warning", 10)
      ]
      ap_messages += [
        ("DAS_settings", 2),
      ]
    elif CP.carFingerprint == CAR.TESLA_MODEL_S_RAVEN:
      # TODO: verify frequencies
      party_messages += [
        ("EPAS_sysStatus", 100),
        ("ESP_private1", 50),
        ("IBST_private2", 50),
      ]

    if CP.carFingerprint in PLATFORM_3Y:
      parsers = {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], party_messages, CANBUS.party),
        Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], ap_messages, CANBUS.autopilot_party),
      }
    elif CP.carFingerprint == CAR.TESLA_MODEL_S_RAVEN:
      pt_messages = [
        ("DI_torque1", 100),
        ("DI_torque2", 100),
        ("DI_state", 10),
      ]

      chassis_messages = [
        ("GTW_carState", 10),
        ("DriverSeat", 20),
      ]

      parsers = {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], party_messages, CANBUS.party),
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.powertrain),
        Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.pt], ap_messages, CANBUS.autopilot_powertrain),
        Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], chassis_messages, CANBUS.chassis),
      }

    return parsers
