import copy

from opendbc.car import Bus, structs

ButtonType = structs.CarState.ButtonEvent.Type

# One event per edge. SET=decelCruise, RES=accelCruise for falling-edge enable.
_BUTTONS = (
  ("RES_BTN", ButtonType.accelCruise),
  ("SET_BTN", ButtonType.decelCruise),
  ("ACC_ON_BTN", ButtonType.cancel),
  ("DEC_DISTANCE_BTN", ButtonType.gapAdjustCruise),
  ("INC_DISTANCE_BTN", ButtonType.altButton2),
)
_BUTTON_FRAME = (
  "SET_BTN", "RES_BTN", "LKAS_ON_BTN", "DEC_DISTANCE_BTN", "INC_DISTANCE_BTN",
  "ACC_ON_BTN", "SET_ME_1_1", "SET_ME_1_2", "COUNTER",
)
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


def cruise_enabled(acc_state: int, lks_enabled: bool) -> bool:
  # ACC_STATE 3/5 + our LKS latch (not camera LKAS_STATE). Same edge as byd_rx_hook.
  return acc_state in (3, 5) and lks_enabled


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CCP = CCP(CP)
    self.lkas_hud = {}
    self.disengage_frames = 0
    self.eps_engaged = True
    self.eps_standby = False
    self.eps_idle = False
    self.eps_target_angle = 0.0
    self.pcm_buttons_stock = {}
    self.pcm_buttons_ts = 0
    self.steer_not_accepted = False
    self.parking_brake = False
    self.epb_on_frames = 0
    self.epb_off_frames = 0
    # Default ON; card.py loads BydLksEnabled before onroad.
    self.lks_enabled = True
    self.lks_btn_last = False
    self.lks_btn_rising = False
    self.camera_lkas_state = 0
    self.camera_acc_state = 0
    self.acc_cmd_stock = {}
    self.acc_hud_stock = {}
    self.pcm_buttons_new = []
    self.button_prev = {name: False for name, _btype in _BUTTONS}
    self.lks_off_alert = 0

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

    # Debounce per STEERING_TORQUE frame to match byd_rx_hook.
    for driver_torque in cp.vl_all["STEERING_TORQUE"]["DRIVER_TORQUE"]:
      if abs(driver_torque) > CCP.STEER_DRIVER_DISENGAGE:
        self.disengage_frames = min(self.disengage_frames + 1, CCP.STEER_DRIVER_DISENGAGE_FRAMES)
      else:
        self.disengage_frames = 0
    ret.steeringDisengage = self.disengage_frames >= CCP.STEER_DRIVER_DISENGAGE_FRAMES

    # After first STEERING_TORQUE sample: engaged=PREP clear; standby=PREP+CRUISE (needs ack).
    if cp.ts_nanos["STEERING_TORQUE"]["LKS_PREPARED"] > 0:
      self.eps_engaged = not cp.vl["STEERING_TORQUE"]["LKS_PREPARED"]
      self.eps_standby = not self.eps_engaged and bool(cp.vl["STEERING_TORQUE"]["CRUISE_ACTIVATED"])
      # Idle = PREP=1 CRUISE=0. Do not pulse LKS in standby (flashes LKAS 4).
      self.eps_idle = (not self.eps_engaged) and (not self.eps_standby)
      self.eps_target_angle = cp.vl["STEERING_TORQUE"]["TARGET_ANGLE"]

    acc_state = int(cp_cam.vl["ACC_HUD_ADAS"]["ACC_STATE"])
    self.camera_acc_state = acc_state
    self.acc_hud_stock = copy.copy(cp_cam.vl["ACC_HUD_ADAS"])
    self.camera_lkas_state = int(cp_cam.vl["LKAS_HUD_ADAS"]["LKAS_STATE"])

    # LKS latch from bus 0 only (ignore bus 2 spoofs).
    self.lks_btn_rising = False
    for btn in cp.vl_all["PCM_BUTTONS"]["LKAS_ON_BTN"]:
      pressed = bool(btn)
      if pressed and not self.lks_btn_last:
        self.lks_enabled = not self.lks_enabled
        self.lks_btn_rising = True
      self.lks_btn_last = pressed
    # Latest 0x3B0 for counter-matched spoofs.
    self.pcm_buttons_stock = copy.copy(cp.vl["PCM_BUTTONS"])
    self.pcm_buttons_ts = cp.ts_nanos["PCM_BUTTONS"]["COUNTER"]
    rows = cp.vl_all["PCM_BUTTONS"]
    n = len(rows["SET_BTN"])
    self.pcm_buttons_new = [{sig: rows[sig][i] for sig in _BUTTON_FRAME} for i in range(n)]

    button_events = []
    for frame in self.pcm_buttons_new:
      for name, btype in _BUTTONS:
        pressed = bool(frame[name])
        # RES also sets SET_BTN; report RES alone.
        if name == "SET_BTN":
          pressed = pressed and not bool(frame["RES_BTN"])
        if pressed != self.button_prev[name]:
          button_events.append(structs.CarState.ButtonEvent(pressed=pressed, type=btype))
          self.button_prev[name] = pressed
    # Long mode: LKS-off must cancel (pcmCruise stays false).
    if self.lks_btn_rising and self.CP.openpilotLongitudinalControl and not self.lks_enabled:
      button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.cancel))
    ret.buttonEvents = button_events

    # SET/RES while LKS off -> invalidLkasSetting alert.
    acc_available = acc_state in (2, 3, 5)
    if self.lks_enabled or not acc_available:
      self.lks_off_alert = 0
    else:
      for event in button_events:
        if not event.pressed and event.type in (ButtonType.accelCruise, ButtonType.decelCruise):
          self.lks_off_alert = self.CCP.LKS_OFF_ALERT_FRAMES
    ret.invalidLkasSetting = self.lks_off_alert > 0
    if self.lks_off_alert > 0:
      self.lks_off_alert -= 1

    # Camera LKAS_STATE is not the LKS switch; fault only if EPS ignores us.
    ret.steerFaultTemporary = self.steer_not_accepted

    # gas / brake
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 0
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"])

    # gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["DRIVE_STATE"]["GEAR"]), GearShifter.unknown)

    # blinkers
    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])

    # BSD sides from APPROACH; RCTA_RIGHT is reverse-only (not RCW / RCTA_RIGHT_2).
    ret.leftBlindspot = cp.vl["BSD_RADAR"]["LEFT_APPROACH"] != 0
    ret.rightBlindspot = (cp.vl["BSD_RADAR"]["RIGHT_APPROACH"] != 0) or bool(cp.vl["BSD_RADAR"]["RCTA_RIGHT"])

    # PCW pre-warning (0x32F byte2 0x36).
    ret.stockFcw = int(cp_cam.vl["PCW_ADAS"]["PCW_STATE"]) == 0x36
    # Hard camera decel while ACC not active.
    self.acc_cmd_stock = copy.copy(cp_cam.vl["ACC_CMD"])
    ret.stockAeb = acc_state not in (3, 5) and cp_cam.vl["ACC_CMD"]["ACCEL_CMD"] < self.CCP.STOCK_AEB_ACCEL

    # doors / belt
    ret.doorOpen = any((
      cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
      cp.vl["METER_CLUSTER"]["TRUNK_OPEN"],
    ))
    ret.seatbeltUnlatched = not bool(cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"])

    # EPB bit 3 applied; d0 also walks 0x09-0x12 in motion - debounce ~70 ms.
    for applied in cp.vl_all["EPB_STATUS"]["EPB_APPLIED"]:
      if applied:
        self.epb_on_frames = min(self.epb_on_frames + 1, self.CCP.EPB_DEBOUNCE_FRAMES)
        self.epb_off_frames = 0
      else:
        self.epb_off_frames = min(self.epb_off_frames + 1, self.CCP.EPB_DEBOUNCE_FRAMES)
        self.epb_on_frames = 0
      if self.epb_on_frames >= self.CCP.EPB_DEBOUNCE_FRAMES:
        self.parking_brake = True
      elif self.epb_off_frames >= self.CCP.EPB_DEBOUNCE_FRAMES:
        self.parking_brake = False
    ret.parkingBrake = self.parking_brake

    # Camera ACC. Lat-only: enable only with LKS latch (ACC can run without OP).
    # ACC_STATE: 0=OFF, 2=ACC_ON, 3=ACC_ACTIVE, 5=FORCE_ACCEL, 7=ERROR
    ret.cruiseState.available = acc_state in (2, 3, 5)
    if self.CP.openpilotLongitudinalControl:
      # Long: camera ACC stays standby (SET blocked); we hold/release ourselves.
      ret.cruiseState.enabled = False
      ret.cruiseState.speed = 0
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = bool(cp_cam.vl["ACC_CMD"]["STANDSTILL_STATE"])
      ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] * CV.KPH_TO_MS
      ret.cruiseState.enabled = cruise_enabled(acc_state, self.lks_enabled)

    self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD_ADAS"])

    return ret

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.lks_enabled:
      return False
    return super().update_button_enable(buttonEvents)

  @staticmethod
  def get_can_parsers(CP):
    body_messages = [
      ("EPB_STATUS", float("nan")),
      ("PCM_BUTTONS", 20),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], body_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
