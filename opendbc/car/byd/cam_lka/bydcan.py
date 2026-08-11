import random

from opendbc.car import rate_limit

SPOOF_TARGET_MIN = 5.0
SPOOF_TARGET_VARIATION_MIN = 4.0
SPOOF_TARGET_NEGATIVE_PROB = 0.3
SPOOF_VARIATION_PROB = 0.2


def byd_checksum(address: int, sig, d: bytearray) -> int:
  """BYD nibble-sum checksum. byte_key 0xAF; checksum in last byte."""
  del address, sig
  byte_key = 0xAF
  sum_first = 0
  sum_second = 0
  for i in range(len(d) - 1):
    sum_first += d[i] >> 4
    sum_second += d[i] & 0xF
  remainder = (sum_second >> 4) & 0xFF
  sum_first += byte_key & 0xF
  sum_second += byte_key >> 4
  inv_first = (-sum_first + 0x9) & 0xF
  inv_second = (-sum_second + 0x9) & 0xF
  return (((inv_first + (5 - remainder)) & 0xF) << 4 | inv_second) & 0xFF


def create_can_steer_command(packer, steer_angle, steer_req, is_standstill, ecu_fault, recovery_btn):
  set_me_xe = 0xE if is_standstill else 0xB
  eps_ok = not steer_req
  if recovery_btn:
    eps_ok = ecu_fault
  # BYD panda safety expects a neutral steering command when steering is inactive.
  steer_angle_cmd = steer_angle if steer_req else 0
  values = {
    "STEER_REQ": steer_req,
    # to recover from ecu fault, it must be momentarily pulled low.
    "EPS_OK": eps_ok,
    "STEER_ANGLE": steer_angle_cmd,
    # must be 0x1 to steer
    "SET_ME_X01": 0x1 if steer_req else 0,
    # 0xB fault lesser, maybe higher value fault lesser, 0xB also seem to have the highest angle limit at high speed.
    "SET_ME_XE": set_me_xe if steer_req else 0,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "UNKNOWN": 2773 if steer_req else 0,
  }

  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)


def create_accel_command(packer, accel, enabled, accel_mult, brake_hold):
  accel = max(min(accel * accel_mult, 30.0), -80.0)
  accel_factor = 12 if accel >= 2 else 5 if accel < 0 else 11
  enabled &= not brake_hold

  if brake_hold:
    accel = 0

  values = {
    "ACCEL_CMD": accel,
    # always 25
    "SET_ME_25_1": 25,
    "SET_ME_25_2": 25,
    "ACC_ON_1": enabled,
    "ACC_ON_2": enabled,
    # some unknown state, 12 when accel, below 11 when braking, 11 when cruising
    "ACCEL_FACTOR": accel_factor if enabled else 0,
    # some unknown state, 0 when not engaged, 3/4 when accel, 8/9 when accel uphill, 1 when braking (all speculation)
    "DECEL_FACTOR": 8 if enabled else 0,
    "SET_ME_X8": 8,
    "SET_ME_1": 1,
    "SET_ME_XF": 0xF,
    "CMD_REQ_ACTIVE_LOW": 0 if enabled else 1,
    "ACC_REQ_NOT_STANDSTILL": enabled,
    "ACC_CONTROLLABLE_AND_ON": enabled,
    "ACC_OVERRIDE_OR_STANDSTILL": brake_hold,
    "STANDSTILL_STATE": brake_hold,
    "STANDSTILL_RESUME": 0,
  }

  return packer.make_can_msg("ACC_CMD", 0, values)


def pack_lkas_hud_status_passthrough(ahb, tsr_status):
  # SET_ME_XFF (8 bits) + TSR_STATUS (3 bits) are continuous in LKAS_HUD_ADAS.
  return ((int(ahb) & 0xFF) << 3) | (int(tsr_status) & 0x7)


def unpack_lkas_hud_status_passthrough(chunk):
  chunk = int(chunk)
  ahb = (chunk >> 3) & 0xFF
  tsr_status = chunk & 0x7
  return ahb, tsr_status


def create_lkas_hud(packer, lat_active, lss_state, lss_alert, tsr,
                    hma, pt2, pt3, pt4, pt5, hud_status_passthrough, lka_on,
                    hand_on_wheel_warning):
  ahb_from_chunk, passthrough_from_chunk = unpack_lkas_hud_status_passthrough(hud_status_passthrough)
  values = {
    "STEER_ACTIVE_ACTIVE_LOW": lka_on,
    "LEFT_LANE_VISIBLE": lat_active,
    "LKAS_ENABLED": lat_active,
    "RIGHT_LANE_VISIBLE": lat_active,
    "LSS_STATE": lss_state,
    "SET_ME_1_2": 1,
    "SETTINGS": lss_alert,
    "TSR_STATUS": passthrough_from_chunk,
    "SET_ME_XFF": ahb_from_chunk,
    "HAND_ON_WHEEL_WARNING": bool(hand_on_wheel_warning),
    "TSR": tsr,
    "HMA": 0, # Could be red lane related (the first bit from the left)
    "PT2": pt2,
    "PT3": pt3,
    "PT4": pt4,
    "PT5": 0, # Could be red lane related
  }

  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)


def send_buttons(packer, state, cancel, bus):
  values = {
    "SET_BTN": state,
    "RES_BTN": state,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "ACC_ON_BTN": cancel,
  }
  return packer.make_can_msg("PCM_BUTTONS", bus, values)


# Module-level state for realistic torque ramp-up simulation
_torque_spoof_state = {
  "current_torque_offset": 0.0,
  "target_torque": 0.0,
  "ramp_rate": 3.0,
  "max_torque": 10.0,
}


def create_steering_torque_spoof_camera(packer, lat_active, main_torque, spoof):
  if spoof:
    if abs(_torque_spoof_state["target_torque"]) < 0.1:
      _torque_spoof_state["target_torque"] = random.uniform(SPOOF_TARGET_MIN, _torque_spoof_state["max_torque"])
      if random.random() < SPOOF_TARGET_NEGATIVE_PROB:
        _torque_spoof_state["target_torque"] = -_torque_spoof_state["target_torque"]

    target = _torque_spoof_state["target_torque"]
    current = _torque_spoof_state["current_torque_offset"]
    ramp = _torque_spoof_state["ramp_rate"]
    if abs(target - current) > 0.1:
      _torque_spoof_state["current_torque_offset"] = rate_limit(target, current, -ramp, ramp)
    else:
      if random.random() < SPOOF_VARIATION_PROB:
        _torque_spoof_state["target_torque"] = random.uniform(
          SPOOF_TARGET_VARIATION_MIN, _torque_spoof_state["max_torque"]
        )
        if random.random() < 0.5:
          _torque_spoof_state["target_torque"] = -_torque_spoof_state["target_torque"]
  else:
    current = _torque_spoof_state["current_torque_offset"]
    if abs(current) > 0.1:
      ramp = _torque_spoof_state["ramp_rate"]
      _torque_spoof_state["current_torque_offset"] = rate_limit(0.0, current, -ramp, ramp)
    else:
      _torque_spoof_state["current_torque_offset"] = 0.0
      _torque_spoof_state["target_torque"] = 0.0

  main_torque += _torque_spoof_state["current_torque_offset"]

  values = {
    "MAIN_TORQUE": main_torque,
    "STATE": 10 if lat_active else 9,
    "CURVATURE": 0,
    "UNKNOWN_TORQUE": 0,
    "STEER_ACTIVE": lat_active,
    "STEER_STATE": 0 if lat_active else 6,
  }

  return packer.make_can_msg("STEERING_TORQUE", 2, values)
