from opendbc.car import Bus, CanBusBase, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import DBC
import copy

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.steer_and_ap_stalk_msg = {}
    self.eps_stock_values = {}
    self.camera_stock_values = {}
    self.longitudinal_stock_values = {}
    self.hud_stock_values = {}

    self.eps_fault_counter = 0
    self.steer_cmd_ignored_counter = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_cam = can_parsers[Bus.cam]
    cp_loopback = can_parsers[Bus.loopback]
    ret = structs.CarState()

    self.steer_and_ap_stalk_msg = copy.copy(cp.vl["STEER_AND_AP_STALK"])
    self.eps_stock_values = copy.copy(cp.vl["RX_STEER_RELATED"])
    self.camera_stock_values = copy.copy(cp_cam.vl["STEER_CMD"])
    self.longitudinal_stock_values = copy.copy(cp_cam.vl["ACC_CMD"])
    self.hud_stock_values = copy.copy(cp_cam.vl["LATERAL_STATE"])

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["FRONT_RIGHT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_RIGHT_WHEEL_SPEED"]
    )

    # CRUISE_STATE_2: 0-2 = deactivated, >2 = active (validated on an H6 PHEV,
    # otaviobonder's haval-new-pcm-signal branch); 0 also observed as fault/off
    cruise_state = cp_cam.vl["ACC"]["CRUISE_STATE_2"]
    ret.accFaulted = bool(cruise_state == 0)
    ret.cruiseState.available = bool(cruise_state > 0)
    ret.cruiseState.enabled = bool(cruise_state > 2)
    ret.cruiseState.speed = cp_cam.vl["ACC"]["ACC_SPEED_SELECTION"]  * CV.KPH_TO_MS
    if not self.CP.openpilotLongitudinalControl:
      ret.cruiseState.speed = -1

    ret.standstill = abs(ret.vEgoRaw) < 1e-3
    ret.gasPressed = cp.vl["CAR_OVERALL_SIGNALS2"]["GAS_POSITION"] > 0
    ret.brakePressed = cp.vl["BRAKE2"]["PEDAL_BRAKE_PRESSED"] != 0
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"] if not ret.brakePressed else 0
    ret.parkingBrake = cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"] == 0

    ret.gearShifter = GearShifter.drive if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 1 else \
                      GearShifter.neutral if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 2 else \
                      GearShifter.reverse if int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]) == 3 else \
                      GearShifter.park

    ret.steeringAngleDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_ANGLE"] * (-1 if cp.vl["STEER_AND_AP_STALK"]["STEERING_DIRECTION"] else 1)
    ret.steeringRateDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_RATE"] * (-1 if (cp.vl["STEER_AND_AP_STALK"]["RATE_DIRECTION"] > 0) else 1)

    # Fault when the EPS ignores our steer command, which is echoed back on the loopback
    # bus (GM pattern). STEER_CMD is sent at 50Hz, so only count on frames where an echo
    # arrived; both thresholds are ~1s.
    if len(cp_loopback.vl_all["STEER_CMD"]["STEER_REQUEST"]) > 0:
      steer_ignored = bool(cp_loopback.vl["STEER_CMD"]["STEER_REQUEST"]) and cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] != 1
      self.steer_cmd_ignored_counter = (self.steer_cmd_ignored_counter + 1) if steer_ignored else 0
    self.eps_fault_counter = (self.eps_fault_counter + 1) if (cp.vl["RX_STEER_RELATED"]["EPS_FAULT_PERMANENT"] == 1) else 0
    ret.steerFaultTemporary = self.steer_cmd_ignored_counter > 50 or self.eps_fault_counter > 100
    ret.steerFaultPermanent = False

    ret.steeringTorque = cp.vl["RX_STEER_RELATED"]["B_RX_DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["RX_STEER_RELATED"]["B_RX_EPS_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > 50

    ret.doorOpen = any([cp.vl["DOOR_DRIVER"]["DOOR_REAR_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_FRONT_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_REAR_LEFT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_DRIVER_OPEN"]])
    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT"]["SEAT_BELT_DRIVER_STATE"])
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"],
                                                                      cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"])
    ret.leftBlindspot = bool(cp.vl["RADAR_BEHIND"]["BSM_LEFT"] > 0)
    ret.rightBlindspot = bool(cp.vl["RADAR_BEHIND"]["BSM_RIGHT"] > 0)

    return ret

  @staticmethod
  def get_can_parsers(CP):
    can_base = CanBusBase(CP, None)
    main_bus = can_base.offset
    adas_bus = can_base.offset + 1
    cam_bus = can_base.offset + 2

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], main_bus),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], adas_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], cam_bus),
      # our own transmitted STEER_CMD, echoed back by the panda; NaN frequency exempts it
      # from alive checks since nothing is echoed until openpilot starts sending
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], [("STEER_CMD", float('nan'))], can_base.offset + 128),
    }
