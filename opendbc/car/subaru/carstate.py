import copy
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.subaru.values import DBC, CanBus, SubaruFlags
from opendbc.car import CanSignalRateCalculator
from opendbc.car.subaru.manual_stats import get_tracker


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["Transmission"]["Gear"]

    self.angle_rate_calulator = CanSignalRateCalculator(50)

    # Manual transmission stats tracker
    if CP.flags & SubaruFlags.MANUAL:
      self.manual_stats = get_tracker()

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_alt = can_parsers[Bus.alt]
    ret = structs.CarState()

    throttle_msg = cp.vl["Throttle"] if not (self.CP.flags & SubaruFlags.HYBRID) else cp_alt.vl["Throttle_Hybrid"]
    ret.gasPressed = throttle_msg["Throttle_Pedal"] > 1e-5
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      ret.brakePressed = cp.vl["Brake_Pedal"]["Brake_Pedal"] > 0
    else:
      cp_brakes = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
      ret.brakePressed = cp_brakes.vl["Brake_Status"]["Brake"] == 1

    cp_es_distance = cp_alt if self.CP.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID) else cp_cam
    if not (self.CP.flags & SubaruFlags.HYBRID):
      eyesight_fault = bool(cp_es_distance.vl["ES_Distance"]["Cruise_Fault"])

      # if openpilot is controlling long, an eyesight fault is a non-critical fault. otherwise it's an ACC fault
      if self.CP.openpilotLongitudinalControl:
        ret.carFaultedNonCritical = eyesight_fault
      else:
        ret.accFaulted = eyesight_fault

    cp_wheels = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
    self.parse_wheel_speeds(ret,
      cp_wheels.vl["Wheel_Speeds"]["FL"],
      cp_wheels.vl["Wheel_Speeds"]["FR"],
      cp_wheels.vl["Wheel_Speeds"]["RL"],
      cp_wheels.vl["Wheel_Speeds"]["RR"],
    )
    ret.standstill = ret.vEgoRaw == 0

    # continuous blinker signals for assisted lane change
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["Dashlights"]["LEFT_BLINKER"],
                                                                      cp.vl["Dashlights"]["RIGHT_BLINKER"])

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSD_RCTA"]["L_ADJACENT"] == 1) or (cp.vl["BSD_RCTA"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSD_RCTA"]["R_ADJACENT"] == 1) or (cp.vl["BSD_RCTA"]["R_APPROACHING"] == 1)

    # Read engine RPM early (needed for manual gear prediction)
    ret.engineRpm = throttle_msg["Engine_RPM"]

    if not self.CP.flags & SubaruFlags.MANUAL:
      cp_transmission = cp_alt if self.CP.flags & SubaruFlags.HYBRID else cp
      can_gear = int(cp_transmission.vl["Transmission"]["Gear"])
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    else:
      # Manual transmission - read clutch and neutral from CAN
      ret.inNeutral = bool(throttle_msg["Neutral"])
      ret.clutchPressed = bool(cp_alt.vl["Cruise_Status"]["Clutch_Depressed"])

      # Predict gear from RPM and speed (no Transmission message on MT)
      # Return 0 when clutch pressed or in neutral - can't determine gear
      if ret.clutchPressed or ret.inNeutral:
        ret.gearActual = 0
      else:
        ret.gearActual = self.manual_stats.predict_gear(ret.engineRpm, ret.vEgo)

      # if ret.inNeutral:
      #   ret.gearShifter = structs.CarState.GearShifter.neutral
      # else:
      ret.gearShifter = structs.CarState.GearShifter.drive

    # Steering
    if not self.CP.flags & SubaruFlags.MANUAL:
      ret.steeringAngleDeg = cp.vl["Steering_Torque"]["Steering_Angle"]

      if not (self.CP.flags & SubaruFlags.PREGLOBAL):
        # ideally we get this from the car, but unclear if it exists. diagnostic software doesn't even have it
        ret.steeringRateDeg = self.angle_rate_calulator.update(ret.steeringAngleDeg, cp.vl["Steering_Torque"]["COUNTER"])

      ret.steeringTorque = cp.vl["Steering_Torque"]["Steer_Torque_Sensor"]
      ret.steeringTorqueEps = cp.vl["Steering_Torque"]["Steer_Torque_Output"]
    else:
      ret.steeringAngleDeg = cp.vl["Brake_Pressure_L_R"]["Steering_Angle"]

    steer_threshold = 75 if self.CP.flags & SubaruFlags.PREGLOBAL else 80
    ret.steeringPressed = abs(ret.steeringTorque) > steer_threshold

    cp_cruise = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
    cp_es_brake = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp_cam

    if self.CP.flags & (SubaruFlags.HYBRID | SubaruFlags.LKAS_ANGLE):
      # ES_DashStatus->Cruise_Activated_Dash is likely intended for the dash display only, as it falls
      # during user gas override and at standstill. ES_Status is missing on hybrid, so we use ES_Brake instead

      # TODO: ES_Brake->Cruise_Activated has been seen staying high when Crosstrek 2025 angle LKAS user pressed
      #  brake while engaged at a stop. ES_Status and ES_DashStatus->Signal7 correctly fell, but is either missing or
      #  always zero on hybrids. Probably need to split angle & hybrid. 0x27 and 0x225 on hybrids may work for them.
      ret.cruiseState.enabled = cp_es_brake.vl["ES_Brake"]['Cruise_Activated'] != 0
      ret.cruiseState.available = cp_cam.vl["ES_DashStatus"]['Cruise_On'] != 0
    else:
      ret.cruiseState.enabled = cp_cruise.vl["CruiseControl"]["Cruise_Activated"] != 0
      ret.cruiseState.available = cp_cruise.vl["CruiseControl"]["Cruise_On"] != 0
    ret.cruiseState.speed = cp_cam.vl["ES_DashStatus"]["Cruise_Set_Speed"] * CV.KPH_TO_MS

    if (self.CP.flags & SubaruFlags.PREGLOBAL and cp.vl["Dash_State2"]["UNITS"] == 1) or \
       (not (self.CP.flags & SubaruFlags.PREGLOBAL) and cp.vl["Dashlights"]["UNITS"] == 1):
      ret.cruiseState.speed *= CV.MPH_TO_KPH

    ret.seatbeltUnlatched = cp.vl["Dashlights"]["SEATBELT_FL"] == 1
    ret.doorOpen = any([cp.vl["BodyInfo"]["DOOR_OPEN_RR"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_RL"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_FR"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_FL"]])
    if not self.CP.flags & SubaruFlags.MANUAL:
      ret.steerFaultPermanent = cp.vl["Steering_Torque"]["Steer_Error_1"] == 1

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      self.cruise_button = cp_cam.vl["ES_Distance"]["Cruise_Button"]
      self.ready = not cp_cam.vl["ES_DashStatus"]["Not_Ready_Startup"]
    else:
      if not self.CP.flags & SubaruFlags.MANUAL:
        ret.steerFaultTemporary = cp.vl["Steering_Torque"]["Steer_Warning"] == 1
      ret.cruiseState.nonAdaptive = cp_cam.vl["ES_DashStatus"]["Conventional_Cruise"] == 1
      ret.cruiseState.standstill = cp_cam.vl["ES_DashStatus"]["Cruise_State"] == 3
      ret.stockFcw = (cp_cam.vl["ES_LKAS_State"]["LKAS_Alert"] == 1) or \
                     (cp_cam.vl["ES_LKAS_State"]["LKAS_Alert"] == 2)

      self.es_lkas_state_msg = copy.copy(cp_cam.vl["ES_LKAS_State"])
      self.es_brake_msg = copy.copy(cp_es_brake.vl["ES_Brake"])

      # TODO: Hybrid cars don't have ES_Distance, need a replacement
      if not (self.CP.flags & SubaruFlags.HYBRID):
        # 8 is known AEB, there are a few other values related to AEB we ignore
        ret.stockAeb = (cp_es_distance.vl["ES_Brake"]["AEB_Status"] == 8) and \
                       (cp_es_distance.vl["ES_Brake"]["Brake_Pressure"] != 0)

        self.es_status_msg = copy.copy(cp_es_brake.vl["ES_Status"])
        self.cruise_control_msg = copy.copy(cp_cruise.vl["CruiseControl"])

    if not (self.CP.flags & SubaruFlags.HYBRID):
      self.es_distance_msg = copy.copy(cp_es_distance.vl["ES_Distance"])

    self.es_dashstatus_msg = copy.copy(cp_cam.vl["ES_DashStatus"])
    if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
      self.es_infotainment_msg = copy.copy(cp_cam.vl["ES_Infotainment"])

    # Update manual transmission stats
    if self.CP.flags & SubaruFlags.MANUAL:
      throttle_pos = throttle_msg["Throttle_Pedal"] / 255.0  # Normalize to 0-1
      if self.manual_stats.frame % 1000 == 0:
        print(f"[MT] frame={self.manual_stats.frame} rpm={ret.engineRpm:.0f} gear={ret.gearActual} speed={ret.vEgo:.1f} throttle={throttle_pos:.2f} clutch={ret.clutchPressed} neutral={ret.inNeutral} lugging={self.manual_stats.is_lugging}", flush=True)
      self.manual_stats.update(
        rpm=ret.engineRpm,
        gear=ret.gearActual,
        speed=ret.vEgo,
        accel=ret.aEgo,
        clutch=ret.clutchPressed,
        neutral=ret.inNeutral,
        throttle=throttle_pos,
      )
      # Log shift quality and lug state for plotjuggler
      ret.shiftSmoothness, ret.shiftGrade = self.manual_stats.get_shift_info()
      ret.isLugging = self.manual_stats.is_lugging

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.main),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.camera),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.alt)
    }
