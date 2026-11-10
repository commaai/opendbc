import copy
import math
from collections import deque

from opendbc.can.parser import CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.leapmotor.values import BUTTON_TAP_GAP_FRAMES, CLUSTER_SPEED_FACTOR, DBC, ENGAGE_TAPS, \
                                         ENGAGE_TAP_WINDOW_FRAMES, STEER_1_DT, CarControllerParams as CCP, LkaState

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


ENGAGE_BUTTON = 1
BUTTONS = {ENGAGE_BUTTON: ButtonType.decelCruise}
CANCEL_BUTTONS = {ENGAGE_BUTTON: ButtonType.cancel}


CAN_REJECTED_BUS = 192
STEER_PRESSED_FRAMES = 5
STEER_FAST_FRAMES = 3

J_SW = 0.060
SG_ACCEL = (5, 0, -3, -4, -3, 0, 5)
SG_ACCEL_SCALE = 0.1 / (42 * STEER_1_DT ** 2)
DRV_AVG_FRAMES = 8


class DriverTorqueFilter:

  def __init__(self):
    self.angles: deque[int] = deque(maxlen=len(SG_ACCEL))
    self.torques: deque[float] = deque(maxlen=len(SG_ACCEL))
    self.recent: deque[float] = deque(maxlen=DRV_AVG_FRAMES)
    self.counter = -1
    self.torque = 0.
    self.avg = 0.

  def update(self, angle: float, torque: float, counter: int) -> float:
    if counter != (self.counter + 1) % 16:
      self.angles.clear()
      self.torques.clear()
    self.counter = counter
    self.angles.append(round(angle * 10))
    self.torques.append(torque)
    if len(self.angles) == len(SG_ACCEL):
      accel = sum(w * a for w, a in zip(SG_ACCEL, self.angles, strict=True)) * SG_ACCEL_SCALE
      self.torque = self.torques[len(SG_ACCEL) // 2] + J_SW * accel
    else:
      self.torque = torque
    if math.isfinite(self.torque):
      self.recent.append(self.torque)
    self.avg = sum(self.recent) / len(self.recent) if self.recent else 0.
    return self.torque


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.stock_lka_cmd = {}
    self.op_enabled = False
    self.eps_fault = False
    self.eps_inactive_frames = 0
    self.lka_active_rejected = 0
    self.driver_torque = DriverTorqueFilter()
    self.strong_frames = 0
    self.steer_angles: list[float] = []
    self.steering_angle_prev = 0.
    self.steering_rate_sign = 1

    self.roller_idle_frames = BUTTON_TAP_GAP_FRAMES + 1
    self.taps = 0
    self.tap_window_frames = 0
    self.engage_btn_prev = 0
    self.engage_cancels = False

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_rejected = can_parsers[Bus.loopback]
    ret = structs.CarState()

    ret.vEgoRaw = cp.vl["VEHICLE_SPEED"]["VEHICLE_SPEED"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * CLUSTER_SPEED_FACTOR
    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = cp.vl["STEER_1"]["STEER_ANGLE"]
    if ret.steeringAngleDeg != self.steering_angle_prev:
      self.steering_rate_sign = 1 if ret.steeringAngleDeg > self.steering_angle_prev else -1
    self.steering_angle_prev = ret.steeringAngleDeg
    ret.steeringRateDeg = self.steering_rate_sign * cp.vl["STEER_1"]["STEER_RATE"]

    ret.steeringTorque = cp.vl["STEER_1"]["STEER_TORQUE_DRIVER"] - CCP.STEER_DRIVER_TORQUE_ZERO
    steer_1 = cp.vl_all["STEER_1"]
    self.steer_angles = list(steer_1["STEER_ANGLE"])
    for angle, torque, counter in zip(steer_1["STEER_ANGLE"], steer_1["STEER_TORQUE_DRIVER"], steer_1["COUNTER"], strict=True):
      self.driver_torque.update(angle, torque - CCP.STEER_DRIVER_TORQUE_ZERO, int(counter))

    self.strong_frames = self.strong_frames + 1 if abs(self.driver_torque.torque) > CCP.STEER_THRESHOLD_FAST else 0
    if self.strong_frames >= STEER_FAST_FRAMES:
      self.steering_pressed_cnt = max(self.steering_pressed_cnt, 2 * STEER_PRESSED_FRAMES)
    ret.steeringPressed = self.update_steering_pressed(abs(self.driver_torque.torque) > CCP.STEER_THRESHOLD, STEER_PRESSED_FRAMES)

    ret.steeringTorqueEps = cp.vl["EPS_FEEDBACK"]["EPS_TORQUE"]

    self.eps_inactive_frames = 0 if cp.vl["STEER_1"]["LKA_ACTIVE"] else self.eps_inactive_frames + 1
    self.lka_active_rejected += sum(state == LkaState.ACTIVE for state in cp_rejected.vl_all["EPS_LKA_CMD"]["LKA_STATE"])
    ret.steerFaultTemporary = self.eps_fault

    ret.gasPressed = cp.vl["ACCEL_PEDAL"]["ACCEL_PEDAL_POS"] > 0
    ret.brakePressed = bool(cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSED"])
    ret.parkingBrake = bool(cp.vl["EPB_1"]["EPB_ENGAGED"])

    gear = int(cp.vl["BRAKE_PEDAL"]["GEAR"])
    if gear == 3:
      ret.gearShifter = GearShifter.drive
    elif gear == 1:
      ret.gearShifter = GearShifter.reverse
    elif gear == 2:
      ret.gearShifter = GearShifter.park if ret.parkingBrake else GearShifter.neutral
    else:
      ret.gearShifter = GearShifter.unknown

    ret.cruiseState.available = True
    ret.buttonEvents = self.update_engage_button(cp_cam)

    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT_STATUS"]["DRIVER_SEATBELT_UNLATCHED"])
    body = cp.vl["BLINKERS_1"]
    ret.doorOpen = any(body[door] for door in ("DOOR_FL", "DOOR_FR", "DOOR_RL", "DOOR_RR"))

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, body["LEFT_LAMP"] or body["LEFT_SWITCH"],
                                                                      body["RIGHT_LAMP"] or body["RIGHT_SWITCH"])

    self.stock_lka_cmd = copy.copy(cp_cam.vl["EPS_LKA_CMD"])

    return ret

  def update_engage_button(self, cp_cam) -> list[structs.CarState.ButtonEvent]:
    """Press the right roller twice to engage, twice again to cancel. Returns that press's button events."""
    pressed = any(cp_cam.vl_all["STEERING_BUTTONS"]["RIGHT_ROLLER_PRESS"])
    new_tap = pressed and self.roller_idle_frames > BUTTON_TAP_GAP_FRAMES
    self.roller_idle_frames = 0 if pressed else self.roller_idle_frames + 1

    self.tap_window_frames += 1
    if self.tap_window_frames > ENGAGE_TAP_WINDOW_FRAMES:
      self.taps = 0
    if new_tap:
      if self.taps == 0:
        self.tap_window_frames = 0
      self.taps += 1

    toggle = self.taps >= ENGAGE_TAPS
    if toggle:
      self.taps = 0
      self.engage_cancels = self.op_enabled
    buttons = CANCEL_BUTTONS if self.engage_cancels else BUTTONS

    engage_btn = ENGAGE_BUTTON if toggle else 0
    events = create_button_events(engage_btn, self.engage_btn_prev, buttons)
    self.engage_btn_prev = engage_btn
    return events

  @staticmethod
  def get_can_parsers(CP):
    rejected = CANParser(DBC[CP.carFingerprint][Bus.pt], [("EPS_LKA_CMD", math.nan)], CAN_REJECTED_BUS)
    for state in rejected.message_states.values():
      state.ignore_counter = True
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [("EPS_FEEDBACK", math.nan)], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [("STEERING_BUTTONS", math.nan)], 2),
      Bus.loopback: rejected,
    }
