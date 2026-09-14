import copy

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.values import DBC, CarControllerParams as CCP
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter

# BYD gear enum from DRIVE_STATE.GEAR
GEAR_MAP = {
  1: GearShifter.park,
  2: GearShifter.reverse,
  3: GearShifter.neutral,
  4: GearShifter.drive,
}


def cruise_enabled(acc_state: int, lkas_state: int) -> bool:
  # ACC_STATE 3/5 = stock ACC active. LKAS_STATE only reads 0 when the driver switches
  # LKS off with 0x3B0 LKAS_ON_BTN; its other values are camera states, not the switch.
  # Mirrored by byd_rx_hook so panda and openpilot enable on the same edge.
  return acc_state in (3, 5) and lkas_state != 0


def lkas_limited(acc_state: int, lkas_state: int) -> bool:
  # Camera LKAS_STATE 4 after ACC-off is a handoff blip, not an OP steer fault.
  return acc_state in (3, 5) and lkas_state == 4


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.lkas_hud = {}
    self.disengage_frames = 0
    self.eps_engaged = True
    self.eps_target_angle = 0.0
    self.steer_not_accepted = False
    self.parking_brake = False
    self.epb_on_frames = 0
    self.epb_off_frames = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # speed
    speed_kph = cp.vl["WHEELSPEED_CLEAN"]["WHEELSPEED_CLEAN"]
    ret.vEgoRaw = speed_kph * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = speed_kph < 0.1
    ret.vEgoCluster = ret.vEgo

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"]
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CCP.STEER_DRIVER_OVERRIDE, 5)

    # Debounced per STEERING_TORQUE frame to match byd_rx_hook, which panda runs on every frame
    for driver_torque in cp.vl_all["STEERING_TORQUE"]["DRIVER_TORQUE"]:
      if abs(driver_torque) > CCP.STEER_DRIVER_DISENGAGE:
        self.disengage_frames = min(self.disengage_frames + 1, CCP.STEER_DRIVER_DISENGAGE_FRAMES)
      else:
        self.disengage_frames = 0
    ret.steeringDisengage = self.disengage_frames >= CCP.STEER_DRIVER_DISENGAGE_FRAMES

    # EPS clears LKS_PREPARED and echoes TARGET_ANGLE while it executes our request.
    # Only trust after the first STEERING_TORQUE frame (parser defaults are 0 / "engaged").
    if cp.ts_nanos["STEERING_TORQUE"]["LKS_PREPARED"] > 0:
      self.eps_engaged = not cp.vl["STEERING_TORQUE"]["LKS_PREPARED"]
      self.eps_target_angle = cp.vl["STEERING_TORQUE"]["TARGET_ANGLE"]

    acc_state = int(cp_cam.vl["ACC_HUD_ADAS"]["ACC_STATE"])
    lkas_state = int(cp_cam.vl["LKAS_HUD_ADAS"]["LKAS_STATE"])

    # LKAS_STATE 4 while ACC is still 3/5 is "LKS is limited". Also fault when we ask
    # and the EPS stays idle for >1 s (carcontroller sets steer_not_accepted).
    ret.steerFaultTemporary = lkas_limited(acc_state, lkas_state) or self.steer_not_accepted

    # gas / brake
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 0
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"])

    # gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["DRIVE_STATE"]["GEAR"]), GearShifter.unknown)

    # blinkers
    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])

    # blind spot monitor
    ret.leftBlindspot = cp.vl["BSD_RADAR"]["LEFT_APPROACH"] != 0
    ret.rightBlindspot = cp.vl["BSD_RADAR"]["RIGHT_APPROACH"] != 0

    # doors / belt
    ret.doorOpen = any((
      cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
      cp.vl["METER_CLUSTER"]["TRUNK_OPEN"],
    ))
    ret.seatbeltUnlatched = not bool(cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"])

    # EPB bit 3 is the applied flag, but d0 also walks 0x09-0x12 during motion.
    # Hold last state until 8 consecutive frames agree (~70 ms at 120 Hz).
    for applied in cp.vl_all["EPB_STATUS"]["EPB_APPLIED"]:
      if applied:
        self.epb_on_frames = min(self.epb_on_frames + 1, CCP.EPB_DEBOUNCE_FRAMES)
        self.epb_off_frames = 0
      else:
        self.epb_off_frames = min(self.epb_off_frames + 1, CCP.EPB_DEBOUNCE_FRAMES)
        self.epb_on_frames = 0
      if self.epb_on_frames >= CCP.EPB_DEBOUNCE_FRAMES:
        self.parking_brake = True
      elif self.epb_off_frames >= CCP.EPB_DEBOUNCE_FRAMES:
        self.parking_brake = False
    ret.parkingBrake = self.parking_brake

    # cruise state: ACC messages come from camera bus on Atto 3
    # ACC_STATE: 0=OFF, 2=ACC_ON (available), 3=ACC_ACTIVE (enabled), 5=FORCE_ACCEL, 7=ERROR
    # LKAS_STATE: 0=LKS switched off, 1=on/passive, 2=active, 3/4=camera states.
    # Follow stock ACC only when LKS is on so ACC can run without engaging OP.
    # Reporting enabled=False while ACC is on must not trip controlsd's cancel spoof.
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] * CV.KPH_TO_MS
    ret.cruiseState.available = acc_state in (2, 3, 5)
    ret.cruiseState.enabled = cruise_enabled(acc_state, lkas_state)
    ret.cruiseState.standstill = bool(cp_cam.vl["ACC_CMD"]["STANDSTILL_STATE"])

    # forward stock LKAS HUD
    self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD_ADAS"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    body_messages = [("EPB_STATUS", float("nan"))]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], body_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
