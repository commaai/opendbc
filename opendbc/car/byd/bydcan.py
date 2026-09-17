from opendbc.car import structs

VisualAlert = structs.CarControl.HUDControl.VisualAlert


def byd_checksum(address: int, sig, d: bytearray) -> int:
  return (~sum(d[:7])) & 0xFF


def create_steering_control(packer, apply_angle: float, lat_active: bool, counter: int, ack: bool = False):
  # Stock saturates the rate limits at +/-299 when engaged, 0 when disengaged.
  # ack is the camera's STEER_REQ=0 / ACTIVE_LOW=0 frame that takes the EPS out of standby.
  rate_limit = 299 if lat_active else 0
  values = {
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_REQ_ACTIVE_LOW": 0 if (lat_active or ack) else 1,
    "STEER_ANGLE": apply_angle,
    "ANGLE_RATE_LIMIT_UPPER": rate_limit,
    "ANGLE_RATE_LIMIT_LOWER": -rate_limit,
    "E2E_ALIVE_1": 1,
    "E2E_ALIVE_2": 1,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "COUNTER": counter,
  }
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)


def create_buttons(packer, stock_values: dict, cancel=False, lkas=False, bus=0):
  # The car's latest 0x3B0 with one button added, COUNTER included: the camera
  # faults ACC on an out of sequence 0x3B0 counter. Cancel is bus 0 ACC_ON_BTN
  # only. Camera LKS neutralize/restore is bus 2 LKAS_ON_BTN only. Never set
  # both: a cancel spoof must not toggle our latch.
  values = {
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    **stock_values,
    "SET_BTN": 0,
    "RES_BTN": 0,
    "DEC_DISTANCE_BTN": 0,
    "INC_DISTANCE_BTN": 0,
    "ACC_ON_BTN": 1 if cancel else 0,
    "LKAS_ON_BTN": 1 if lkas else 0,
  }
  return packer.make_can_msg("PCM_BUTTONS", bus, values)


def create_lkas_hud(packer, counter: int, stock_lkas_hud: dict, hud_control, lat_active: bool):
  # Called for the whole engagement. Cluster wheel follows our lateral state,
  # not the camera: 2 while we steer, 1 while we only heartbeat.
  # Cluster nag bits are ours: do not pass the camera's hands-off timer through.
  # steerRequired is TAKE CONTROL / DM; SET_ME_50=2 is the chime stage, not 3
  # (stock's last step before it drops ACC).
  values = {**stock_lkas_hud, "COUNTER": counter}
  values["LKS_MODE"] = 2  # green lane line icon
  values["LKAS_STATE"] = 2 if lat_active else 1
  # LANE_STATE: 0=Grey, 1=Green, 2=Orange
  values["LEFT_LANE_STATE"] = 1 if hud_control.leftLaneVisible else 0
  values["RIGHT_LANE_STATE"] = 1 if hud_control.rightLaneVisible else 0

  if hud_control.leftLaneDepart:
    values["LEFT_LANE_STATE"] = 2
  if hud_control.rightLaneDepart:
    values["RIGHT_LANE_STATE"] = 2

  take_control = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
  values["HANDS_ON_WHEEL_REQ"] = 1 if take_control else 0
  values["SET_ME_50"] = 2 if hud_control.visualAlert == VisualAlert.steerRequired else 0

  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)
