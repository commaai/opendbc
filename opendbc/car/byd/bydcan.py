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


def create_acc_cmd(packer, accel: float, long_active: bool, hold: bool, resume: bool, counter: int):
  # Idle 64 64 64 80 50 00. Active sets both ACC_ON bits. Hold is byte 5 0x31.
  # Stock drives off from a hold with STANDSTILL_RESUME=1, STANDSTILL_STATE=0 and
  # a positive ACCEL_CMD, and keeps RESUME up until about 2 m/s.
  hold = long_active and hold
  resume = long_active and resume and not hold
  values = {
    "ACCEL_CMD": accel if long_active else 0.0,
    "ACC_ON_1": 1 if long_active else 0,
    "ACC_ON_2": 1 if long_active else 0,
    "SET_ME_25_1": 25,
    "SET_ME_25_2": 25,
    "DECEL_FACTOR": 3 if long_active else 0,
    "SET_ME_X8": 8,
    "ACCEL_FACTOR": 12 if long_active else 0,
    "CMD_REQ_ACTIVE_LOW": 0 if long_active else 1,
    "SET_ME_1": 1,
    "STANDSTILL_RESUME": 1 if resume else 0,
    "STANDSTILL_STATE": 1 if hold else 0,
    "ACC_REQ_NOT_STANDSTILL": 1 if (long_active and not hold) else 0,
    "ACC_CONTROLLABLE_AND_ON": 1 if long_active else 0,
    "ACC_OVERRIDE_OR_STANDSTILL": 1 if hold else 0,
    "COUNTER": counter,
    "SET_ME_XF": 0xF,
  }
  return packer.make_can_msg("ACC_CMD", 0, values)


def create_acc_cmd_passthrough(packer, stock_values: dict, counter: int):
  values = {**stock_values, "COUNTER": counter, "SET_ME_XF": 0xF}
  return packer.make_can_msg("ACC_CMD", 0, values)


def create_acc_hud(packer, stock_values: dict, counter: int, acc_state: int, set_speed_kph: float, gap_bars: int,
                   lead_visible: bool):
  # byte2 is 0x04 with ACC_STATE in bits 3-5 and ACC_ON1 in bit 6.
  # ACC_ON2 is the middle state bit; writing both keeps the packer from clearing it.
  # SET_DISTANCE 1 is closest, 4 is farthest. 0 keeps the camera's gap.
  # LEAD_VISIBLE follows the lead; the camera never sets it in standby.
  if gap_bars <= 0:
    distance = stock_values.get("SET_DISTANCE", 3)
  else:
    distance = min(max(int(gap_bars), 1), 4)
  values = {
    "SET_ME_B2_LO": 4,
    "SET_ME_B3": 1,
    "SET_ME_B4": 0xF4,
    "SET_ME_XFF": 0xFF,
    "SET_ME_XF": 0xF,
    **stock_values,
    "COUNTER": counter,
    "ACC_STATE": acc_state,
    "ACC_ON1": 1 if acc_state in (2, 3, 5) else 0,
    "ACC_ON2": (acc_state >> 1) & 1,
    "SET_SPEED": max(0.0, min(float(set_speed_kph), 127.5)),
    "SET_DISTANCE": distance,
    "LEAD_VISIBLE": 1 if lead_visible else 0,
  }
  return packer.make_can_msg("ACC_HUD_ADAS", 0, values)


def create_buttons_relay(packer, stock_values: dict, lkas: bool = False):
  # Stock 0x3B0 on bus 2 with SET/RES/distance cleared. COUNTER and ACC_ON stay
  # as the driver pressed them, so the camera counter stays in sequence. LKAS_ON
  # is ours: the camera drops a second frame with the same counter.
  values = {
    **stock_values,
    "SET_BTN": 0,
    "RES_BTN": 0,
    "DEC_DISTANCE_BTN": 0,
    "INC_DISTANCE_BTN": 0,
    "LKAS_ON_BTN": 1 if lkas else 0,
  }
  return packer.make_can_msg("PCM_BUTTONS", 2, values)


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


def create_lkas_hud(packer, counter: int, stock_lkas_hud: dict, hud_control, lat_active: bool, lks_on: bool = True):
  # Called for the whole engagement. Cluster wheel follows our lateral state,
  # not the camera: 2 while we steer, 1 while we only heartbeat.
  # Cluster nag bits are ours: do not pass the camera's hands-off timer through.
  # steerRequired is TAKE CONTROL / DM; SET_ME_50=2 is the chime stage, not 3
  # (stock's last step before it drops ACC).
  # LKS off: we still own 0x316 until panda's 2 s HUD hold, so paint the
  # wheel off immediately. LKAS_STATE 1 is the standby wheel, not off.
  # LKS_MODE / lane bits are the car (LKA) icon -- leave the camera's.
  values = {**stock_lkas_hud, "COUNTER": counter}
  if not lks_on:
    values["LKAS_STATE"] = 0
    values["HANDS_ON_WHEEL_REQ"] = 0
    values["SET_ME_50"] = 0
    return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)

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
