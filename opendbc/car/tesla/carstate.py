import copy
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import CAR, DBC, CANBUS, GEAR_MAP, PLATFORM_3Y

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])

    self.hands_on_level = 0
    self.das_control = None

  def update(self, can_parsers) -> structs.CarState:
    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    cp_pt = can_parsers[Bus.pt]
    ret = structs.CarState()

    # Vehicle speed
    if self.CP.carFingerprint in PLATFORM_3Y:
      ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    else:
      ret.vEgoRaw = cp_party.vl["ESP_private1"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    if self.CP.carFingerprint in PLATFORM_3Y:
      pedal_status = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"]
    else:
      pedal_status = cp_pt.vl["DI_torque1"]["DI_pedalPos"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = (pedal_status > 0)

    # Brake pedal
    ret.brake = 0

    if self.CP.carFingerprint in PLATFORM_3Y:
      ret.brakePressed = cp_party.vl["IBST_status"]["IBST_driverBrakeApply"] == 2
    else:
      ret.brakePressed = cp_party.vl["IBST_private2"]["IBST_brakePedalApplied"] == 1

    # Steering wheel
    epas_name = "EPAS3S" if self.CP.carFingerprint in PLATFORM_3Y else "EPAS"
    epas_status = cp_party.vl[f"{epas_name}_sysStatus"]
    self.hands_on_level = epas_status[f"{epas_name}_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status[f"{epas_name}_internalSAS"]
    if self.CP.carFingerprint in PLATFORM_3Y:
      ret.steeringRateDeg = -cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    else:
      ret.steeringRateDeg = -cp_party.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status[f"{epas_name}_torsionBarTorque"]

    ret.steeringPressed = self.hands_on_level > 0
    eac_status = self.can_define.dv[f"{epas_name}_sysStatus"][f"{epas_name}_eacStatus"].get(int(epas_status[f"{epas_name}_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # Cruise state
    di_state = cp_party.vl["DI_state"] if self.CP.carFingerprint in PLATFORM_3Y else cp_pt.vl["DI_state"]
    # TODO: this needs a different can_define
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(di_state["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(di_state["DI_speedUnits"]), None)

    ret.cruiseState.enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    if speed_units == "KPH":
      ret.cruiseState.speed = di_state["DI_digitalSpeed"] * CV.KPH_TO_MS
    elif speed_units == "MPH":
      ret.cruiseState.speed = di_state["DI_digitalSpeed"] * CV.MPH_TO_MS
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1

    # Blinkers
    ret.leftBlinker = cp_party.vl["UI_warning"]["leftBlinkerOn"] != 0
    ret.rightBlinker = cp_party.vl["UI_warning"]["rightBlinkerOn"] != 0

    # Seatbelt
    ret.seatbeltUnlatched = cp_party.vl["UI_warning"]["buckleStatus"] != 1

    # Blindspot
    ret.leftBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
    ret.rightBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0

    # AEB
    ret.stockAeb = cp_ap_party.vl["DAS_control"]["DAS_aebEvent"] == 1

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_party.vl["DAS_control"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    party_messages = []
    if CP.carFingerprint in PLATFORM_3Y:
      party_messages += [
        ("DI_speed", 50),
        ("DI_systemStatus", 100),
        ("IBST_status", 25),
        ("DI_state", 10),
        ("EPAS3S_sysStatus", 100),
        ("UI_warning", 10)
      ]
    elif CP.carFingerprint == CAR.TESLA_MODEL_S_RAVEN:
      # TODO: verify frequencies
      party_messages += [
        ("STW_ANGLHP_STAT", 100),
        ("EPAS_sysStatus", 100),
        ("ESP_private1", 50),
        ("IBST_private2", 50),
      ]

    ap_party_messages = [
      ("DAS_control", 25),
      ("DAS_status", 2),
    ]

    parser = None
    if CP.carFingerprint in PLATFORM_3Y:
      messages += [
        ("SCCM_steeringAngleSensor", 100),
      ]
      parser = CANParser(DBC[CP.carFingerprint][Bus.party], party_messages, CANBUS.autopilot_party)
    elif CP.carFingerprint == CAR.TESLA_MODEL_S_RAVEN:
      parser = CANParser(DBC[CP.carFingerprint][Bus.pt], party_messages, CANBUS.autopilot_powertrain)

    parsers = {
      Bus.party: parser,
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], ap_party_messages, CANBUS.autopilot_party),
    }

    if CP.carFingerprint == CAR.TESLA_MODEL_S_RAVEN:
      pt_messages = [
        ("DI_torque1", 100),
        ("DI_state", 10),
      ]
      parsers[Bus.pt] = CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.powertrain),

    return parsers
