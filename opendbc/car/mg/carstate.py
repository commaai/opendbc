from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mg.values import DBC, GEAR_MAP
from opendbc.car.common.conversions import Conversions as CV

GearShifter = structs.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_radar = can_parsers[Bus.radar]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["SCS_HSC2_FrP15"]["VehSpdAvgDrvnHSC2"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = abs(ret.vEgoRaw) < 0.01

    # Gas pedal
    ret.gasPressed = cp.vl["GW_HSC2_HCU_FrP00"]["EPTAccelActuPosHSC2"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["EHBS_HSC2_FrP00"]["BrkPdlAppdHSC2"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["SAS_HSC2_FrP00"]["StrgWhlAngHSC2"]
    ret.steeringRateDeg = cp.vl["SAS_HSC2_FrP00"]["StrgWhlAngGrdHSC2"]
    ret.steeringTorque = cp.vl["EPS_HSC2_FrP03"]["DrvrStrgDlvrdToqHSC2"]
    ret.steeringTorqueEps = cp.vl["EPS_HSC2_FrP03"]["ChLKARespToqHSC2"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 1.0, 5)

    ret.steerFaultTemporary = cp_cam.vl["FVCM_HSC2_FrP02"]["LDWSysFltStsHSC2"] != 0  # TODO: validate

    # Cruise state
    ret.cruiseState.enabled = cp.vl["RADAR_HSC2_FrP00"]["ACCSysSts_RadarHSC2"] in (2, 3)  # Active, Override
    ret.cruiseState.available = True
    ret.cruiseState.standstill = False  # TODO
    ret.cruiseState.speed = cp.vl["RADAR_HSC2_FrP02"]["ACCDrvrSelTrgtSpd_RadarHSC2"] * CV.KPH_TO_MS

    ret.accFaulted = cp_cam.vl["FVCM_HSC2_FrP02"]["TJAICASysFltStsHSC2"] != 0  # TODO: validate

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["GW_HSC2_ECM_FrP04"]["TrEstdGearHSC2"]), GearShifter.unknown)

    # Doors
    ret.doorOpen = False  # TODO

    # Blinkers
    ret.leftBlinker = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 1
    ret.rightBlinker = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 2

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["GW_HSC2_SDM_FrP00"]["DrvrSbltAtcHSC2"] != 1

    # Blindspot
    # ret.leftBlindspot = False
    # ret.rightBlindspot = False

    # AEB
    ret.stockAeb = False

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.radar: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
