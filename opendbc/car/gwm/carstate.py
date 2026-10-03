import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import DBC, CarControllerParams

ButtonType = structs.CarState.ButtonEvent.Type

EPS_ACTIVE = 1


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"]

    self.distance_button = 0
    self.steer_ignored_cnt = 0
    self.eps_fault_cnt = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_loopback = can_parsers[Bus.loopback]
    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["FRONT_RIGHT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_RIGHT_WHEEL_SPEED"],
    )
    ret.standstill = ret.vEgoRaw < 1e-3

    ret.gasPressed = cp.vl["CAR_OVERALL_SIGNALS2"]["GAS_POSITION"] > 0
    ret.brakePressed = cp.vl["BRAKE2"]["PEDAL_BRAKE_PRESSED"] == 1
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"])))

    stalk = cp.vl["STEER_AND_AP_STALK"]
    ret.steeringAngleDeg = stalk["STEERING_ANGLE"] * (-1 if stalk["STEERING_DIRECTION"] else 1)
    ret.steeringRateDeg = stalk["STEERING_RATE"] * (-1 if stalk["RATE_DIRECTION"] else 1)
    ret.steeringTorque = cp.vl["RX_STEER_RELATED"]["B_RX_DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["RX_STEER_RELATED"]["B_RX_EPS_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD

    # Our STEER_CMD is echoed back on the loopback bus. Fault if the EPS doesn't go active for ~1s while we request,
    # only counting on cycles that received an echo since STEER_CMD is sent at half the update rate
    if len(cp_loopback.vl_all["STEER_CMD"]["STEER_REQUEST"]):
      steer_ignored = cp_loopback.vl["STEER_CMD"]["STEER_REQUEST"] == 1 and cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] != EPS_ACTIVE
      self.steer_ignored_cnt = self.steer_ignored_cnt + 1 if steer_ignored else 0
    self.eps_fault_cnt = self.eps_fault_cnt + 1 if cp.vl["RX_STEER_RELATED"]["EPS_FAULT_PERMANENT"] == 1 else 0
    ret.steerFaultTemporary = self.steer_ignored_cnt > 50 or self.eps_fault_cnt > 100

    # 1-2: standby, 3: engaged, 5: engaged with driver overriding
    cruise_state = cp_cam.vl["ACC"]["CRUISE_STATE_2"]
    ret.cruiseState.available = cruise_state != 0
    ret.cruiseState.enabled = cruise_state in (3, 5)
    ret.cruiseState.speed = cp_cam.vl["ACC"]["ACC_SPEED_SELECTION"] * CV.KPH_TO_MS

    ret.doorOpen = any([cp.vl["DOOR_DRIVER"]["DOOR_REAR_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_FRONT_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_REAR_LEFT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_DRIVER_OPEN"]])
    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["SEAT_BELT_DRIVER_STATE"] == 1
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"],
                                                                      cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"])
    ret.leftBlindspot = cp.vl["RADAR_BEHIND"]["BSM_LEFT"] != 0
    ret.rightBlindspot = cp.vl["RADAR_BEHIND"]["BSM_RIGHT"] != 0

    prev_distance_button = self.distance_button
    self.distance_button = int(stalk["AP_REDUCE_DISTANCE_COMMAND"] or stalk["AP_INCREASE_DISTANCE_COMMAND"])
    ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    # stock messages to modify and send
    self.stalk_stock_values = copy.copy(stalk)
    self.eps_stock_values = copy.copy(cp.vl["RX_STEER_RELATED"])
    self.steer_stock_values = copy.copy(cp_cam.vl["STEER_CMD"])
    self.acc_stock_values = copy.copy(cp_cam.vl["ACC_CMD"])
    self.hud_stock_values = copy.copy(cp_cam.vl["LATERAL_STATE"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
      # NaN frequency skips the alive check, nothing is echoed until openpilot starts sending
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], [("STEER_CMD", float('nan'))], 128),
    }
