# from opendbc.car.can_definitions import CanData


def compute_set_distance(state):
    return 1
    # if state == SetDistance.aggressive:
    #     return 2
    # elif state == SetDistance.normal:
    #     return 1
    # else:
    #     return 0

# not an actual known crc function, reverse engineered


def byte_crc4_linear_inverse(byte_list):
    return (-1 * sum(byte_list) + 0x9) & 0xf


def byd_checksum(byte_key, dat):

    second_bytes = [byte & 0xf for byte in dat]
    remainder = sum(second_bytes) >> 4
    second_bytes.append(byte_key >> 4)

    first_bytes = [byte >> 4 for byte in dat]
    first_bytes.append(byte_key & 0xf)

    return (((byte_crc4_linear_inverse(first_bytes) + (-1*remainder + 5)) << 4) + byte_crc4_linear_inverse(second_bytes)) & 0xff


def create_can_steer_command(packer, steer_angle, steer_req, is_standstill, raw_cnt):

    set_me_xe = 0xB
    if is_standstill:
        set_me_xe = 0xE

    values = {
        "STEER_REQ": steer_req,
        "STEER_REQ_ACTIVE_LOW": not steer_req,
        "STEER_ANGLE": steer_angle * 1.02,     # desired steer angle
        "SET_ME_X01": 0x1 if steer_req else 0,  # must be 0x1 to steer
        "SET_ME_XE": set_me_xe if steer_req else 0,
        # 0xB fault lesser, maybe higher value fault lesser, 0xB also seem to have the highest angle limit at high speed.
        "COUNTER": raw_cnt,
        "SET_ME_FF": 0xFF,
        "SET_ME_F": 0xF,
        "SET_ME_1_1": 1,
        "SET_ME_1_2": 1,
    }

    dat = packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)[2]
    crc = byd_checksum(0xaf, str(dat)[:-1])
    values["CHECKSUM"] = crc
    return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)


def create_accel_command(packer, accel, enabled, brake_hold, raw_cnt):
    accel = max(min(accel * 16.67, 30), -50)

    values = {
        "ACCEL_CMD": accel,
        "SET_ME_25_1": 25,                     # always 25
        "SET_ME_25_2": 25,                     # always 25
        "COUNTER": raw_cnt,
        "ACC_ON_1": enabled,
        "ACC_ON_2": enabled,
        # the higher the value, the more powerful the accel
        "ACCEL_FACTOR": 14 if enabled else 0,
        # the lower the value, the more powerful the decel
        "DECEL_FACTOR": 1 if enabled else 0,
        "SET_ME_X8": 8,
        "SET_ME_1": 1,
        "SET_ME_XF": 0xF,
        "CMD_REQ_ACTIVE_LOW": 0 if enabled else 1,
        "ACC_REQ_NOT_STANDSTILL": enabled,
        "ACC_CONTROLLABLE_AND_ON": enabled,
        "ACC_OVERRIDE_OR_STANDSTILL": 0,       # use this to apply brake hold
        "STANDSTILL_STATE": 0,                 # TODO integrate vEgo check
        "STANDSTILL_RESUME": 0,                # TODO integrate buttons
    }

    dat = packer.make_can_msg("ACC_CMD", 0, values)[2]
    crc = byd_checksum(0xaf, dat[:-1])
    values["CHECKSUM"] = crc
    return packer.make_can_msg("ACC_CMD", 0, values)

# 50hz


def create_lkas_hud(packer, enabled, lss_state, lss_alert, tsr, ahb, passthrough, hma, pt2, pt3, pt4, pt5, lka_on, raw_cnt):

    values = {
        "STEER_ACTIVE_ACTIVE_LOW": lka_on,  # not enabled,
        "STEER_ACTIVE_1_1": enabled and lka_on,  # Left lane visible
        # steering wheel between lanes icon, lkas active
        "STEER_ACTIVE_1_2": enabled and lka_on,
        "STEER_ACTIVE_1_3": enabled and lka_on,  # Right lane visible
        "LSS_STATE": lss_state,
        "SET_ME_1_2": 1,
        "SETTINGS": lss_alert,
        "SET_ME_X5F": ahb,
        "SET_ME_XFF": passthrough,
        # TODO integrate warning signs when steer limited
        "HAND_ON_WHEEL_WARNING": 0,
        "TSR": tsr,
        "HMA": hma,
        "PT2": pt2,
        "PT3": pt3,
        "PT4": pt4,
        "PT5": pt5,
        "COUNTER": raw_cnt,
    }

    dat = packer.make_can_msg("LKAS_HUD_ADAS", 0, values)[2]
    crc = byd_checksum(0xaf, str(dat)[:-1])
    values["CHECKSUM"] = crc
    return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)


def send_buttons(packer, state, count):
    """Spoof ACC Button Command."""
    values = {
        "SET_BTN": state,
        "RES_BTN": state,
        "SET_ME_1_1": 1,
        "SET_ME_1_2": 1,
        "COUNTER": count,
    }
    dat = packer.make_can_msg("PCM_BUTTONS", 0, values)[2]
    crc = byd_checksum(0xaf, str(dat)[:-1])
    values["CHECKSUM"] = crc
    return packer.make_can_msg("PCM_BUTTONS", 0, values)
