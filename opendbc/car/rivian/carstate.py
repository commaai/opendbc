import copy
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.rivian.values import DBC, GEAR_MAP, RivianFlags
from opendbc.car.common.conversions import Conversions as CV

GearShifter = structs.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.last_speed = 30

    self.acm_lka_hba_cmd: dict | None = None
    self.sccm_wheel_touch: dict | None = None
    self.vdm_adas_status: list[dict] | None = None

    self.v_cruise = 0
    self.prev_stalk_enable_adj = 0
    self.prev_stalk_cancel_res = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_adas = can_parsers[Bus.adas]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_Status"]["ESP_Vehicle_Speed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = abs(ret.vEgoRaw) < 0.01
    conversion = CV.KPH_TO_MS if cp_adas.vl["Cluster"]["Cluster_Unit"] == 0 else CV.MPH_TO_MS
    ret.vEgoCluster = cp_adas.vl["Cluster"]["Cluster_VehicleSpeed"] * conversion

    # Gas pedal
    ret.gasPressed = cp.vl["VDM_PropStatus"]["VDM_AcceleratorPedalPosition"] > 0

    # Brake pedal
    ret.brakePressed = cp.vl["iBESP2"]["iBESP2_BrakePedalApplied"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["EPAS_AdasStatus"]["EPAS_InternalSas"]
    ret.steeringRateDeg = cp.vl["EPAS_AdasStatus"]["EPAS_SteeringAngleSpeed"]
    ret.steeringTorque = cp.vl["EPAS_SystemStatus"]["EPAS_TorsionBarTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 1.0, 5)

    ret.steerFaultTemporary = cp.vl["EPAS_AdasStatus"]["EPAS_EacErrorCode"] != 0

    # Cruise state
    speed = min(int(cp_adas.vl["ACM_tsrCmd"]["ACM_tsrSpdDisClsMain"]), 85)
    self.last_speed = speed if speed != 0 else self.last_speed
    ret.cruiseState.enabled = cp_cam.vl["ACM_Status"]["ACM_FeatureStatus"] == 1

    is_metric = cp_adas.vl["Cluster"]["Cluster_Unit"] == 0
    speed_conv = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS

    stalk_enable_adj = cp.vl["VDM_AdasStalk"]["VDM_AdasStalkAccEnableAdj"]
    stalk_cancel_res = cp.vl["VDM_AdasStalk"]["VDM_AdasStalkAccCancelRes"]

    if ret.cruiseState.enabled:
      if self.v_cruise == 0:
        current_speed = round(ret.vEgoCluster / speed_conv)
        self.v_cruise = max(20 if not is_metric else 30, current_speed)

      if stalk_enable_adj != self.prev_stalk_enable_adj:
        if stalk_enable_adj == 1:
          self.v_cruise += 1
        elif stalk_enable_adj == 2:
          self.v_cruise += 5
        elif stalk_enable_adj == 3:
          self.v_cruise -= 1
        elif stalk_enable_adj == 4:
          self.v_cruise -= 5

        min_speed = 20 if not is_metric else 30
        max_speed = 85 if not is_metric else 140
        self.v_cruise = max(min_speed, min(self.v_cruise, max_speed))
    else:
      self.v_cruise = 0

    self.prev_stalk_enable_adj = stalk_enable_adj
    self.prev_stalk_cancel_res = stalk_cancel_res

    ret.cruiseState.speed = self.v_cruise * speed_conv
    if not self.CP.openpilotLongitudinalControl:
      ret.cruiseState.speed = -1
    ret.cruiseState.available = True  # cp.vl["VDM_AdasSts"]["VDM_AdasInterfaceStatus"] == 1
    ret.cruiseState.standstill = cp.vl["VDM_AdasSts"]["VDM_AdasVehicleHoldStatus"] == 1

    # ACM_Status->ACM_FaultSupervisorState normally 1, appears to go to 3 when either:
    # 1. car in park/not in drive (normal)
    # 2. something (message from another ECU) ACM relies on is faulty
    #  * ACM_FaultStatus will stay 0 since ACM itself isn't faulted
    # TODO: ACM_FaultStatus hasn't been seen high yet, but log anyway
    ret.accFaulted = (cp_cam.vl["ACM_Status"]["ACM_FaultStatus"] == 1 or
                      # VDM_AdasFaultStatus=Brk_Intv is the default for some reason
                      # VDM_AdasFaultStatus=Cntr_Fault isn't fully understood, but we've seen it in the wild
                      # VDM_AdasFaultStatus=Imps_Cmd was seen when sending it rapidly changing ACC enable commands, or when ACC command drops out
                      cp.vl["VDM_AdasSts"]["VDM_AdasFaultStatus"] in (2, 3))  # 2=Cntr_Fault, 3=Imps_Cmd

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["VDM_PropStatus"]["VDM_Prndl_Status"]), GearShifter.unknown)

    # Doors and seatbelt
    # GEN2 has no CAN signal for these, but stock ACC already handles disengaging
    # door locks prevent opening while driving
    # on standstill, stock ACC disengages when a door is opened or seatbelt is unbuckled
    if not (self.CP.flags & RivianFlags.GEN2):
      ret.doorOpen = any(cp_adas.vl["IndicatorLights"][door] != 2 for door in ("RearDriverDoor", "FrontPassengerDoor", "DriverDoor", "RearPassengerDoor"))
      ret.seatbeltUnlatched = cp.vl["RCM_Status"]["RCM_Status_IND_WARN_BELT_DRIVER"] != 0

    # Blinkers
    ret.leftBlinker = cp_adas.vl["IndicatorLights"]["TurnLightLeft"] in (1, 2)
    ret.rightBlinker = cp_adas.vl["IndicatorLights"]["TurnLightRight"] in (1, 2)

    # Blindspot
    # ret.leftBlindspot = False
    # ret.rightBlindspot = False

    # AEB
    ret.stockAeb = cp_cam.vl["ACM_AebRequest"]["ACM_EnableRequest"] != 0

    # Messages needed by carcontroller
    self.acm_lka_hba_cmd = copy.copy(cp_cam.vl["ACM_lkaHbaCmd"])
    if not (self.CP.flags & RivianFlags.GEN2):
      self.sccm_wheel_touch = copy.copy(cp.vl["SCCM_WheelTouch"])
    # This message can lag and send two messages at once, make sure we forward all of them
    adas_status_msgs = cp.vl_all["VDM_AdasSts"]
    self.vdm_adas_status = [dict(zip(adas_status_msgs, vals, strict=True)) for vals in zip(*adas_status_msgs.values(), strict=True)]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
