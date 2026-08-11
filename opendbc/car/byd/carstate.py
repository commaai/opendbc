from opendbc.can.parser import CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, BUTTONS, LKAS_HUD_PASSTHROUGH

ButtonType = structs.CarState.ButtonEvent.Type

# UNKNOWN (bytes 0-1), SET_ME_X01 and SET_ME_XE of STEERING_MODULE_ADAS are *constant* for
# the whole of a camera steering episode — the camera holds bytes 0-2 at 2b 55 eb from the
# first STEER_REQ frame to the last, and 00 00 e0 while idle. Measured over the 2026-08-09
# drive: 1111 of the 1121 frames the camera sent with STEER_REQ set carried exactly this
# triple, the remaining ten being the one- or two-frame ramp in and out of an episode.
#
# They are NOT a 20-bit sequence to be continued, and treating them as one is what broke
# lateral control: openpilot latched the camera's value and added a fixed step every frame,
# which walks SET_ME_XE through all sixteen values and SET_ME_X01 through all four. The car
# accepts a few tens of seconds of that and then drops the entire stock ADAS — ACC_HUD_ADAS
# ACC_ON1/ACC_ON2 and every ACC_CMD engagement bit go to zero together, taking openpilot's
# cruiseState with them. Only ever send a triple the camera itself has been seen to send.
STEER_TEMPLATE_FIELDS = ("UNKNOWN", "SET_ME_X01", "SET_ME_XE")

# The camera's steering frame, captured on-car: bytes 0-2 = 2b 55 eb.
STEER_TEMPLATE_DEFAULT = {"UNKNOWN": 2773, "SET_ME_X01": 1, "SET_ME_XE": 0xB}

# The camera ramps in over a frame or two at the start of an episode, so only adopt a
# template once it has held still — otherwise we can latch a transitional value.
STEER_TEMPLATE_STABLE_FRAMES = 5

# ACC_CMD arrives at 50Hz with no counter/checksum validation in the parser, and a
# single corrupted frame must not drop cruiseState.enabled for one frame — enough to fire
# pcmDisable and drop panda's controls_allowed. Require a few consecutive frames to drop.
CRUISE_DROP_FRAMES = 5


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.button_states = {button.event_type: False for button in BUTTONS}
    self.lkas_hud = dict.fromkeys(LKAS_HUD_PASSTHROUGH, 0)
    # the camera's own steering frame, held constant; None until it has steered once
    self.steer_template = None
    self._template_candidate = None
    self._template_frames = 0
    self.acc_off_frames = 0
    self.cruise_enabled_last = False
    self.prev_angle = 0.0

  def update_steer_template(self, cam_steer) -> None:
    """
        Latch the camera's steering frame while the camera is the one driving the EPS, so
        that what we transmit is byte-identical to what this car's own ADAS transmits.

        The fields are constant for the whole of a steering episode, so only adopt a value
        once it has held still — the camera spends a frame or two ramping in and out, and
        latching one of those transitional frames would have us send, forever, a triple the
        car never sends.
        """
    if not cam_steer["STEER_REQ"]:
      self._template_candidate = None
      self._template_frames = 0
      return

    template = {k: int(cam_steer[k]) for k in STEER_TEMPLATE_FIELDS}
    if not any(template.values()):  # the idle frame, sent during the ramp in
      return

    if template == self._template_candidate:
      self._template_frames += 1
    else:
      self._template_candidate = template
      self._template_frames = 1

    if self._template_frames >= STEER_TEMPLATE_STABLE_FRAMES:
      self.steer_template = template

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]       # bus 0: car-side chassis CAN
    cp_cam = can_parsers[Bus.cam]  # bus 2: camera-side chassis CAN
    ret = structs.CarState.new_message()

    # --- Steering ---
    ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"]
    ret.steeringRateDeg = (ret.steeringAngleDeg - self.prev_angle) / DT_CTRL
    self.prev_angle = ret.steeringAngleDeg
    # DRIVER_EPS_TORQUE (byte 2 of STEER_MODULE_2): actual column torque sensor, raw 0–255 unsigned.
    # MAIN_TORQUE (STEERING_TORQUE 0x1FC) is total EPS motor output — NOT driver input.
    ret.steeringTorque = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"]
    ret.steeringTorqueEps = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
    ret.steeringPressed = ret.steeringTorque > 80  # raw threshold; observed max ~52 during normal turns

    # --- Pedals ---
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 1.0
    brake_pedal = cp.vl["PEDAL"]["BRAKE_PEDAL"]  # physical, DBC factor 0.01 already applied
    # BRAKE_PEDAL is the only signal that tracks the driver's foot. Verified against a
    # controlled pedal capture and against 3.6h of driving:
    #   - DRIVE_STATE.BRAKE_PRESSED (DBC bit 37) is dead — byte 4 is a constant 0x0C.
    #   - PEDAL_PRESSED_ACTIVE_LOW is the brake-light switch: 86% of its assertions on
    #     the road happened while the camera's ACC was commanding decel, not the driver.
    ret.brakePressed = brake_pedal > 0.03  # raw > 3, matches BYD_BRAKE_THRESHOLD in panda

    # --- Gear ---
    gear_map = {
      1: structs.CarState.GearShifter.park,
      2: structs.CarState.GearShifter.reverse,
      4: structs.CarState.GearShifter.drive,
    }
    ret.gearShifter = gear_map.get(int(cp.vl["DRIVE_STATE"]["GEAR"]),
                                   structs.CarState.GearShifter.unknown)

    # --- Wheel speeds ---
    # DBC factor 0.1 gives km/h, but BYD ATTO3 India raw values read ~32.5% high
    # vs odometer at steady-state. Correction: 40/53. Verify with GPS if re-calibrating.
    _SPD_CORR = 40.0 / 53.0
    fl = cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"] * _SPD_CORR / 3.6
    fr = cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"] * _SPD_CORR / 3.6
    rl = cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"] * _SPD_CORR / 3.6
    # WHEELSPEED_BR (bits 48-63): byte 7 is a constant status byte (0x41),
    # NOT the high byte of the wheel speed. DBC wrongly declares it as 16-bit.
    # Until the DBC is corrected, derive RR from the other three wheels.
    rr = (fl + fr + rl) / 3.0
    ret.wheelSpeeds.fl = fl
    ret.wheelSpeeds.fr = fr
    ret.wheelSpeeds.rl = rl
    ret.wheelSpeeds.rr = rr
    ret.vEgoRaw = (fl + fr + rl) / 3.0
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.05

    # --- Cruise / ACC --- (read from camera bus 2 — native source)
    # ACC_HUD_ADAS.ACC_ON1/ACC_ON2 are the ACC *main switch / standby* state, NOT
    # engagement. Proven on the 2026-08-07 drive: 25 min of stop-and-go with the driver
    # braking 28% of the time and both bits set throughout. The real engaged flag is in
    # ACC_CMD: CMD_REQ_ACTIVE_LOW == 0 (with ACC_ON_1/2 set) only while the stock ACC is
    # actively commanding. Using standby as "enabled" trapped openpilot after every brake
    # tap: cruiseState.enabled never dropped, so pcmEnable never saw a rising edge again.
    acc_main = bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON1"]) or bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON2"])
    acc_engaged = (bool(cp_cam.vl["ACC_CMD"]["ACC_ON_1"]) and bool(cp_cam.vl["ACC_CMD"]["ACC_ON_2"])
                   and not bool(cp_cam.vl["ACC_CMD"]["CMD_REQ_ACTIVE_LOW"]))

    # Debounce the drop: a single corrupted frame must not disengage.
    if acc_engaged:
      self.acc_off_frames = 0
    else:
      self.acc_off_frames += 1
    if self.acc_off_frames == 0:
      self.cruise_enabled_last = True
    elif self.acc_off_frames >= CRUISE_DROP_FRAMES:
      self.cruise_enabled_last = False

    ret.cruiseState.enabled = self.cruise_enabled_last
    # available is the ACC main switch, not the engaged state — aliasing the two made
    # every disengage also raise wrongCarMode and block re-engagement.
    ret.cruiseState.available = acc_main
    # DBC SET_SPEED factor 0.5 already applied (gives km/h); convert to m/s
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] / 3.6
    ret.cruiseState.standstill = False

    # We take over LKAS entirely: panda blocks the camera's steering command and we
    # send our own, so the stock system is never a competing controller.
    ret.stockLkas = False
    self.lkas_hud = {k: cp_cam.vl["LKAS_HUD_ADAS"][k] for k in LKAS_HUD_PASSTHROUGH}

    self.update_steer_template(cp_cam.vl["STEERING_MODULE_ADAS"])

    # --- Safety ---
    ret.seatbeltUnlatched = not bool(cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"])
    ret.doorOpen = any([
      cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
    ])

    # --- Blinkers ---
    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])

    # --- Button events ---
    button_events = []
    for button in BUTTONS:
      if button.can_addr not in ("PCM_BUTTONS", "STALKS"):
        continue
      msg_val = bool(cp.vl[button.can_addr][button.can_msg])
      if msg_val != self.button_states[button.event_type]:
        event = structs.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = msg_val
        button_events.append(event)
        self.button_states[button.event_type] = msg_val
    ret.buttonEvents = button_events

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Bus 0: messages sent by car ECUs (EPS, BCM, wheel speed, pedals, etc.)
    pt_messages = [
      ("STEER_MODULE_2", 0),
      ("STEERING_TORQUE", 0),
      ("PEDAL", 0),
      ("DRIVE_STATE", 0),
      ("WHEEL_SPEED", 0),
      ("METER_CLUSTER", 0),
      ("PCM_BUTTONS", 0),
      ("STALKS", 0),
    ]
    # Bus 2: messages sent by the ADAS camera module
    cam_messages = [
      ("ACC_HUD_ADAS", 0),
      ("ACC_CMD", 0),
      ("LKAS_HUD_ADAS", 0),
      ("STEERING_MODULE_ADAS", 0),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, 2),
    }
