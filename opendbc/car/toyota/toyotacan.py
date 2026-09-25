import copy

from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.structs import CarParams

SteerControlType = CarParams.SteerControlType


def create_steer_command(packer, steer, steer_req):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)


def create_lta_steer_command(packer, steer_control_type, steer_angle, steer_req, frame, torque_wind_down):
  """Creates a CAN message for the Toyota LTA Steer Command."""

  values = {
    "COUNTER": frame + 128,
    "SETME_X1": 1,  # suspected LTA feature availability
    # 1 for TSS 2.5 cars, 3 for TSS 2.0. Send based on whether we're using LTA for lateral control
    "SETME_X3": 1 if steer_control_type == SteerControlType.angle else 3,
    "PERCENTAGE": 100,
    "TORQUE_WIND_DOWN": torque_wind_down,
    "ANGLE": 0,
    "STEER_ANGLE_CMD": steer_angle,
    "STEER_REQUEST": steer_req,
    "STEER_REQUEST_2": steer_req,
    "CLEAR_HOLD_STEERING_ALERT": 0,
  }
  return packer.make_can_msg("STEERING_LTA", 0, values)


def create_lta_steer_command_2(packer, frame):
  values = {
    "COUNTER": frame + 128,
  }
  return packer.make_can_msg("STEERING_LTA_2", 0, values)


def create_accel_command(packer, accel, pcm_cancel, permit_braking, standstill_req, lead, acc_type, fcw_alert, distance):
  # TODO: find the exact canceling bit that does not create a chime
  values = {
    "ACCEL_CMD": accel,
    "ACC_TYPE": acc_type,
    "DISTANCE": distance,
    "MINI_CAR": lead,
    "PERMIT_BRAKING": permit_braking,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
    "ALLOW_LONG_PRESS": 1,
    "ACC_CUT_IN": fcw_alert,  # only shown when ACC enabled
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)


def create_accel_command_2(packer, accel):
  values = {
    "ACCEL_CMD": accel,
  }
  return packer.make_can_msg("ACC_CONTROL_2", 0, values)


def create_tss3_brake_cancel_command(packer, stock_brake, bus):
  """Clone live 0x101 state and assert only the native brake-cancel bit."""
  values = {
    "SET_ME_1": stock_brake["SET_ME_1"],
    "BRAKE_PRESSED": 1,
    "BRAKE_BYTE_1": stock_brake["BRAKE_BYTE_1"],
    "BRAKE_BYTE_3": stock_brake["BRAKE_BYTE_3"],
  }
  return packer.make_can_msg("BRAKE_MODULE", bus, values)


def create_tss3_control_request_values(stock_request, lat_active: bool, angle_raw: int, long_active: bool, accel: float,
                                       set_speed_kph: float, request_sequence: int):
  """CONTROL_REQUEST signals, with stock longitudinal only the lateral request and sequence of the FRC's are replaced."""
  lateral = {
    "LATERAL_REQUEST_PINION_ANGLE": angle_raw * 0.001000121519,
    "LATERAL_REQUEST_ID": 11 if lat_active else 0,  # LTA/LCA
    "LATERAL_ASSIST_GAIN": 1.0 if lat_active else 0.5,
    "LATERAL_DAMPING_GAIN": 0,
    "REQUEST_SEQUENCE": request_sequence,
  }
  if stock_request is not None:
    return {**stock_request, **lateral}

  accel = accel if long_active else 0.0
  return {
    "CRUISE_OPERATING_LATCH": 1,
    "SET_ME_1": 1,
    "LONGITUDINAL_REQUEST_ID_UPPER": 11,
    "LONGITUDINAL_ALLOCATION_METHOD_UPPER": 1,  # engine and brake
    "LONGITUDINAL_REQUEST_ID_LOWER": 17,
    "LONGITUDINAL_ALLOCATION_METHOD_LOWER": 3,  # brake only
    "LONGITUDINAL_REQUEST_ACCEL_UPPER": accel,
    "LONGITUDINAL_REQUEST_ACCEL_LOWER": accel,
    "SET_SPEED": min(max(round(set_speed_kph), 0), 255),
    "SET_ME_X7FFF": 0x7FFF,
    "SET_ME_X7FFF_2": 0x7FFF,
    "CRUISE_STATE_MIRROR": 3,
    "CRUISE_REQUEST_ACTIVE": 1,
    **lateral,
  }


def create_tss3_signer_requests(packer, bus: int, signer_sequence: int, application: bytes):
  # four fragments of the 28-byte request, headers alternate the low and high nibble of the sequence
  nibbles = (signer_sequence & 0xF, signer_sequence >> 4)
  msgs = []
  for fragment in range(4):
    data = application[fragment * 7:(fragment + 1) * 7]
    msgs.append(packer.make_can_msg("SIGNER_REQUEST", bus, {
      "HEADER": 0x80 | (fragment << 4) | nibbles[fragment % 2],
      "DATA_1": int.from_bytes(data[:3], "big"),
      "DATA_2": int.from_bytes(data[3:], "big"),
    }))
  return msgs


def create_tss3_signer_arm(packer, bus: int, arm: bool):
  # arm: panda publishes openpilot's CONTROL_REQUEST instead of the FRC's, release: the FRC's again
  return packer.make_can_msg("SIGNER_REQUEST", bus, {"HEADER": 0x07, "DATA_1": 0xC9A800 | int(arm)})


def create_tss3_lkas_hud(packer, bus, stock_hud, left_line: bool, right_line: bool, lat_active: bool, steer_alert: bool):
  values = copy.copy(stock_hud)

  # forward startup and unknown states untouched
  if values["LTA_MODE"] in (0x10, 0x12, 0x14) and values["LTA_INDICATOR"] in (0, 1, 2):
    line = 4 if lat_active else 1
    if left_line == right_line:
      values["LANE_LINE_1"] = values["LANE_LINE_2"] = line if left_line else 2
    else:
      # which line is left is unknown, so only update the stock visible lines
      for sig in ("LANE_LINE_1", "LANE_LINE_2"):
        if values[sig] in (1, 4):
          values[sig] = line

    values.update({
      "LTA_MODE": 0x14 if lat_active else 0x12,
      "LTA_INDICATOR": 1 if lat_active else 2,
      "HANDS_ON_WARNING": 3 if steer_alert else 0,
      "HANDS_ON_WARNING_2": 0,
    })

  return packer.make_can_msg("LKAS_HUD", bus, values)


def create_pcs_commands(packer, accel, active, mass):
  values1 = {
    "COUNTER": 0,
    "FORCE": round(min(accel, 0) * mass * 2),
    "STATE": 3 if active else 0,
    "BRAKE_STATUS": 0,
    "PRECOLLISION_ACTIVE": 1 if active else 0,
  }
  msg1 = packer.make_can_msg("PRE_COLLISION", 0, values1)

  values2 = {
    "DSS1GDRV": min(accel, 0),     # accel
    "PCSALM": 1 if active else 0,  # goes high same time as PRECOLLISION_ACTIVE
    "IBTRGR": 1 if active else 0,  # unknown
    "PBATRGR": 1 if active else 0, # noisy actuation bit?
    "PREFILL": 1 if active else 0, # goes on and off before DSS1GDRV
    "AVSTRGR": 1 if active else 0,
  }
  msg2 = packer.make_can_msg("PRE_COLLISION_2", 0, values2)

  return [msg1, msg2]


def create_acc_cancel_command(packer):
  values = {
    "GAS_RELEASED": 0,
    "CRUISE_ACTIVE": 0,
    "ACC_BRAKING": 0,
    "ACCEL_NET": 0,
    "CRUISE_STATE": 0,
    "CANCEL_REQ": 1,
  }
  return packer.make_can_msg("PCM_CRUISE", 0, values)


def create_fcw_command(packer, fcw):
  values = {
    "PCS_INDICATOR": 1,  # PCS turned off
    "FCW": fcw,
    "SET_ME_X20": 0x20,
    "SET_ME_X10": 0x10,
    "PCS_OFF": 1,
    "PCS_SENSITIVITY": 0,
  }
  return packer.make_can_msg("PCS_HUD", 0, values)


def create_ui_command(packer, steer, chime, left_line, right_line, left_lane_depart, right_lane_depart, enabled, stock_lkas_hud):
  values = {
    "TWO_BEEPS": chime,
    "LDA_ALERT": steer,
    "RIGHT_LINE": 3 if right_lane_depart else 1 if right_line else 2,
    "LEFT_LINE": 3 if left_lane_depart else 1 if left_line else 2,
    "BARRIERS": 1 if enabled else 0,

    # static signals
    "SET_ME_X02": 2,
    "SET_ME_X01": 1,
    "LKAS_STATUS": 1,
    "REPEATED_BEEPS": 0,
    "LANE_SWAY_FLD": 7,
    "LANE_SWAY_BUZZER": 0,
    "LANE_SWAY_WARNING": 0,
    "LDA_FRONT_CAMERA_BLOCKED": 0,
    "TAKE_CONTROL": 0,
    "LANE_SWAY_SENSITIVITY": 2,
    "LANE_SWAY_TOGGLE": 1,
    "LDA_ON_MESSAGE": 0,
    "LDA_MESSAGES": 0,
    "LDA_SA_TOGGLE": 1,
    "LDA_SENSITIVITY": 2,
    "LDA_UNAVAILABLE": 0,
    "LDA_MALFUNCTION": 0,
    "LDA_UNAVAILABLE_QUIET": 0,
    "ADJUSTING_CAMERA": 0,
    "LDW_EXIST": 1,
  }

  # lane sway functionality
  # not all cars have LKAS_HUD — update with camera values if available
  if len(stock_lkas_hud):
    values.update({s: stock_lkas_hud[s] for s in [
      "LANE_SWAY_FLD",
      "LANE_SWAY_BUZZER",
      "LANE_SWAY_WARNING",
      "LANE_SWAY_SENSITIVITY",
      "LANE_SWAY_TOGGLE",
    ]})

  return packer.make_can_msg("LKAS_HUD", 0, values)


def toyota_checksum(address: int, sig, d: bytearray) -> int:
  s = len(d)
  addr = address
  while addr:
    s += addr & 0xFF
    addr >>= 8
  for i in range(len(d) - 1):
    s += d[i]
  return s & 0xFF


def toyota_e2e_checksum(address: int, sig, d: bytearray) -> int:
  # AUTOSAR E2E profile 5: CRC-16/CCITT over the payload after the checksum, then the address as the data ID
  crc = 0xFFFF
  for byte in (*d[2:], address & 0xFF, (address >> 8) & 0xFF):
    crc = ((crc << 8) ^ CRC16_XMODEM[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
  return crc
