from opendbc.car.byd.values import BydSteerStates

def create_can_steer_command(packer, steer_angle, steer_req, is_standstill, steer_state):

  set_me_xe = 0xE if is_standstill else 0xB

  values = {
    "STEER_REQ": steer_req,
    # to recover from ecu fault, it must be momentarily pulled low.
    "STEER_STATE": BydSteerStates.STEER_ENABLED if steer_req else steer_state,
    "STEER_ANGLE": steer_angle,
    # 0xB fault lesser, maybe higher value fault lesser, 0xB also seem to have the highest angle limit at high speed.
    "SET_ME_XE": set_me_xe if steer_req else 0,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "UNKNOWN": 2773 if steer_req else 0,
    }

  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)

def create_accel_command(packer, accel, enabled, brake_hold):
  accel = max(min(accel * 13, 30), -50)
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

def create_lkas_hud(packer, hud_pt, settings_pt, enabled, lka_on):

  values = {
    "ADAS_SETTINGS_PT": settings_pt,
    "HUD_PASSTHROUGH": hud_pt,
    # TODO integrate warning signs when steer limited
    "HAND_ON_WHEEL_WARNING": 0,
    "LKAS_ENABLED_ACTIVE_LOW": lka_on,
    "LKAS_ACTIVE": enabled and lka_on,
  }

  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)

def send_buttons(packer, state):
  values = {
    "SET_BTN": state,
    "RES_BTN": state,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
  }
  return packer.make_can_msg("PCM_BUTTONS", 0, values)

