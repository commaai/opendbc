import numpy as np

from opendbc.car.crc import CRC8J1850

# 64-byte messages are split into 8-byte blocks, each protected by its own CRC8 (first byte) and
# counter (last byte). The counter counts 0-14 and the CRC uses a different xor_out per block.
COUNTER_CYCLE = 15


def gwm_checksum(dat: bytes, xor_out: int) -> int:
  crc = 0
  for b in dat:
    crc = CRC8J1850[crc ^ b]
  return crc ^ xor_out


def steer_cmd_basic_checksum(dat: bytes) -> int:
  torque = ((dat[12] & 0x7F) << 3) | (dat[13] >> 5)  # raw TORQUE_CMD, 10-bit two's complement
  positive_torque = 0 < torque < 0x200
  counter = dat[15] & 0xF
  steer_request = (dat[15] >> 5) & 0x1
  return (28 - (steer_request * 8) - counter - positive_torque) & 0x1F


def create_steer_command(packer, stock_values, steer: int, steer_req: bool):
  counter = (stock_values["COUNTER"] + 1) % COUNTER_CYCLE
  values = {
    "STEER_REQUEST": steer_req,
    "SET_ME_X01": 1,
    "TORQUE_CMD": steer,
    "TORQUE_REFLECTED": -steer,
    "INVERT_DIRECTION": steer_req and steer > 0,
    "COUNTER": counter,
    "BYPASS_ME": stock_values["BYPASS_ME"],
    "COUNTER_X34": counter,
  }

  dat = packer.make_can_msg("STEER_CMD", 0, values)[1]
  values["BASIC_CHECKSUM"] = steer_cmd_basic_checksum(dat)
  dat = packer.make_can_msg("STEER_CMD", 0, values)[1]
  values["CRC_X9B"] = gwm_checksum(dat[9:16], 0x9B)
  values["CRC_X34"] = gwm_checksum(dat[17:24], 0x34)
  return packer.make_can_msg("STEER_CMD", 0, values)


def create_longitudinal_command(packer, stock_values, accel: float, active: bool, standstill: bool):
  values = {s: stock_values[s] for s in (
    "BRAKE_OR_GAS_REQ",
    "BYPASSME_1",
    "SPEED_REAL",
    "COUNTER_BRAKE",
    "BYPASSME_2",
    "COUNTER_X8A",
    "BYPASS_ACC1",
    "BYPASS_ACC2",
    "COUNTER_ACC",
    "STANDSTILL_1",
    "STANDSTILL_2",
    "STANDSTILL_3",
  )}
  values |= {
    "BRAKE_CMD": 0,
    "GAS_CMD": 0,
  }

  # accel is normalized to [-1, 1]
  if active and accel < 0:
    values |= {
      "BRAKE_OR_GAS_REQ": 13,
      "BRAKE_CMD": accel * (107 - 41) - 41,
      "STANDSTILL_1": standstill,
      "STANDSTILL_2": 3 if standstill else 4,
      "STANDSTILL_3": 0 if standstill else 1,
    }
  elif active:
    values |= {
      "BRAKE_OR_GAS_REQ": 12,
      "GAS_CMD": np.interp(accel, [0.25, 1], [0, 4577]),
      "STANDSTILL_1": 0,
      "STANDSTILL_2": 4,
      "STANDSTILL_3": 1,
    }

  dat = packer.make_can_msg("ACC_CMD", 0, values)[1]
  values["CRC_BRAKE_0xEF"] = gwm_checksum(dat[9:16], 0xEF)
  values["CRC_X8A"] = gwm_checksum(dat[17:24], 0x8A)
  values["CRC_ACC_0x87"] = gwm_checksum(dat[25:32], 0x87)
  return packer.make_can_msg("ACC_CMD", 0, values)


def create_eps_feedback(packer, stock_values, driver_torque: float):
  # EPS feedback relayed to the camera, with driver torque replaced so the camera's hands-on-wheel check is satisfied
  values = {s: stock_values[s] for s in (
    "A_CRC_X61",
    "A_BYPASSME_2",
    "A_RX_STEER_REQUESTED",
    "A_BYPASSME_1",
    "A_COUNTER",
    "B_BYPASSME_1",
    "B_BYPASSME_2",
    "B_RX_EPS_TORQUE",
    "B_COUNTER",
    "B_BYPASSME_4",
    "EPS_FAULT_PERMANENT",
    "B_BYPASSME_3",
  )}
  values["B_RX_DRIVER_TORQUE"] = driver_torque

  dat = packer.make_can_msg("RX_STEER_RELATED", 2, values)[1]
  values["B_CRC_X61"] = gwm_checksum(dat[9:16], 0x61)
  return packer.make_can_msg("RX_STEER_RELATED", 2, values)


def create_buttons_command(packer, stock_values, cancel: bool):
  values = {s: stock_values[s] for s in (
    "STEERING_ANGLE",
    "STEERING_DIRECTION",
    "STEERING_RATE",
    "RATE_DIRECTION",
  )}
  values |= {
    "AP_CANCEL_COMMAND": cancel,
    # one ahead of the car's message so the camera acts on ours
    "COUNTER": (stock_values["COUNTER"] + 1) % COUNTER_CYCLE,
  }

  dat = packer.make_can_msg("STEER_AND_AP_STALK", 2, values)[1]
  values["CRC_X2D"] = gwm_checksum(dat[1:8], 0x2D)
  return packer.make_can_msg("STEER_AND_AP_STALK", 2, values)


def create_hud_command(packer, stock_values, lat_active: bool):
  values = {s: stock_values[s] for s in (
    "BYPASSME_1",
    "BYPASSME_2",
    "BY_PASSME",
    "COUNTER",
    "BYPASSME_3",
    "BYPASSME_4",
    "BYPASSME_5",
    "BYPASSME_6",
    "BYPASSME_7",
    "CRUISE_STATE",
  )}
  values["LKAS_STATE"] = 5 if lat_active else stock_values["LKAS_STATE"]

  dat = packer.make_can_msg("LATERAL_STATE", 0, values)[1]
  values["CRC_X66"] = gwm_checksum(dat[17:24], 0x66)
  return packer.make_can_msg("LATERAL_STATE", 0, values)
