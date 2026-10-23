from opendbc.car import structs
from opendbc.car.crc import CRC8J1850, mk_crc8_fun

VisualAlert = structs.CarControl.HUDControl.VisualAlert

_crc8_j1850 = mk_crc8_fun(CRC8J1850)
BYD_RADAR_K1 = {
  0x280: 0x7C, 0x281: 0x08, 0x282: 0x59, 0x283: 0x2D, 0x284: 0xB1,
  0x285: 0xC5, 0x286: 0x26, 0x287: 0x52, 0x288: 0xCE, 0x289: 0xBA,
}


def byd_checksum(address: int, sig, d: bytearray) -> int:
  return (~sum(d[:7])) & 0xFF


def byd_radar_checksum(address: int, sig, d: bytearray) -> int:
  k1 = BYD_RADAR_K1.get(address)
  if k1 is None:
    i = sig.start_bit // 8
    return d[i] if i < len(d) else 0
  if sig.start_bit == 0:
    return _crc8_j1850(d[1:32]) ^ k1
  return _crc8_j1850(d[33:64]) ^ k1 ^ 0x3A


def create_steering_control(packer, apply_angle: float, lat_active: bool, counter: int, ack: bool = False):
  # Stock rate limits +/-299 engaged, 0 disengaged. ack = camera standby wake (REQ=0, ACTIVE_LOW=0).
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
  # Idle 64 64 64 80 50 00. Hold byte5 0x31. Stock leave-hold: RESUME=1, STATE=0, clear RESUME ~2 m/s.
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
                   lead_visible: bool, gas_override: bool = False, lead_follow_status: int = 0):
  # SET_DISTANCE 1=closest..4=farthest; 0 keeps camera gap. ACC_ON1/ON2 both set with ACC_STATE.
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
  # Override: stock holds byte3 at 0x21.
  if gas_override:
    values["ACC_OVERRIDE"] = 1
    values["SET_ME_B3"] = 1
  # LEAD_SHADING inverse of leadFollowStatus (wire green=3).
  if lead_follow_status > 0:
    values["LEAD_SHADING"] = 4 - lead_follow_status
  return packer.make_can_msg("ACC_HUD_ADAS", 0, values)


def create_buttons_relay(packer, stock_values: dict, lkas: bool = False):
  # Bus 2 relay: clear SET/RES/distance; keep COUNTER/ACC_ON. Optional LKAS_ON pulse.
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
  # Keep stock COUNTER. Cancel = bus 0 ACC_ON only; LKS pulse = bus 2 LKAS_ON only (never both).
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
  # Own 0x316: wheel 2=steer / 1=idle / 0=LKS off. STEER_TAKEOVER=2 for steerRequired (not 3).
  values = {**stock_lkas_hud, "COUNTER": counter}
  if not lks_on:
    values["LKAS_STATE"] = 0
    values["HANDS_ON_WHEEL_REQ"] = 0
    values["STEER_TAKEOVER"] = 0
    return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)

  values["LKS_MODE"] = 2  # green lane line icon
  values["LKAS_STATE"] = 2 if lat_active else 1
  # LANE_STATE: 0=Grey, 1=Green, 2=Orange
  values["LEFT_LANE_STATE"] = 1 if hud_control.leftLaneVisible else 0
  values["RIGHT_LANE_STATE"] = 1 if hud_control.rightLaneVisible else 0

  if hud_control.leftLaneVisible and hud_control.leftLaneDepart:
    values["LEFT_LANE_STATE"] = 2
  if hud_control.rightLaneVisible and hud_control.rightLaneDepart:
    values["RIGHT_LANE_STATE"] = 2

  take_control = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
  values["HANDS_ON_WHEEL_REQ"] = 1 if take_control else 0
  values["STEER_TAKEOVER"] = 2 if hud_control.visualAlert == VisualAlert.steerRequired else 0

  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)
