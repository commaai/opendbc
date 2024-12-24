import copy
from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.rivian.values import DBC, GEAR_MAP
from opendbc.car.common.conversions import Conversions as CV

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.last_speed = 30

    # Needed by carcontroller
    self.acm_lka_hba_cmd = None

  def update(self, cp, cp_cam, cp_adas, *_) -> structs.CarState:
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESPiB1"]["ESPiB1_VehicleSpeed"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    # Gas pedal
    pedal_status = cp.vl["VDM_PropStatus"]["VDM_AcceleratorPedalPosition"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = (pedal_status > 0)

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["iBESP2"]["iBESP2_BrakePedalApplied"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["EPAS_AdasStatus"]["EPAS_InternalSas"]
    ret.steeringRateDeg = cp.vl["EPAS_AdasStatus"]["EPAS_SteeringAngleSpeed"]
    ret.steeringTorque = cp.vl["EPAS_SystemStatus"]["EPAS_TorsionBarTorque"]
    ret.steeringPressed = abs(ret.steeringTorque) > 1.0

    ret.steerFaultTemporary = cp.vl["EPAS_AdasStatus"]["EPAS_EacErrorCode"] != 0

    # Cruise state
    speed = min(int(cp_adas.vl["ACM_tsrCmd"]["ACM_tsrSpdDisClsMain"]), 85)
    self.last_speed = speed if speed != 0 else self.last_speed
    ret.cruiseState.enabled = cp_cam.vl["ACM_Status"]["ACM_FeatureStatus"] != 0
    ret.cruiseState.speed = self.last_speed * CV.MPH_TO_MS  # detected speed limit
    ret.cruiseState.available = True # cp.vl["VDM_AdasSts"]["VDM_AdasInterfaceStatus"] == 1
    ret.cruiseState.standstill = cp.vl["VDM_AdasSts"]["VDM_AdasAccelRequestAcknowledged"]

    # Gear
    ret.gearShifter = GEAR_MAP[int(cp.vl["VDM_PropStatus"]["VDM_Prndl_Status"])]

    # Doors
    ret.doorOpen = False

    # Blinkers
    ret.leftBlinker = cp_adas.vl["IndicatorLights"]["TurnLightLeft"] in (1, 2)
    ret.rightBlinker = cp_adas.vl["IndicatorLights"]["TurnLightRight"] in (1, 2)

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["RCM_Status"]["RCM_Status_IND_WARN_BELT_DRIVER"] != 0

    # Blindspot
    # ret.leftBlindspot = False
    # ret.rightBlindspot = False

    # AEB
    ret.stockAeb = cp_cam.vl["ACM_AebRequest"]["ACM_EnableRequest"] != 0

    # Messages needed by carcontroller
    self.acm_lka_hba_cmd = copy.copy(cp_cam.vl["ACM_lkaHbaCmd"])

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      # sig_address, frequency
      ("ESPiB1", 50),
      ("VDM_PropStatus", 50),
      ("iBESP2", 50),
      ("EPAS_AdasStatus", 100),
      ("EPAS_SystemStatus", 100),
      ("RCM_Status", 8),
      ("VDM_AdasSts", 100)
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("ACM_longitudinalRequest", 100),
      ("ACM_AebRequest", 100),
      ("ACM_Status", 100),
      ("ACM_lkaHbaCmd", 100)
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], messages, 2)

  @staticmethod
  def get_adas_can_parser(CP):
    messages = [
      ("IndicatorLights", 10),
      ("ACM_tsrCmd", 10),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], messages, 1)
