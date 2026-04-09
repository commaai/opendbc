import numpy as np
from opendbc.car import CanBusBase


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def radar(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2


def create_steer_command(packer, CAN: CanBus, camera_stock_values, steer: float, steer_req: bool):
  steer = int(steer)
  values = {
    "STEER_REQUEST": 1 if steer_req else 0,
    "SET_ME_X01": 1,
    "TORQUE_CMD": steer,
    "TORQUE_REFLECTED": -steer,
    "INVERT_DIRECTION": 1 if (steer > 0 and steer_req) else 0,
    "COUNTER": (camera_stock_values["COUNTER"] + 1) % 16,
    "BYPASS_ME": camera_stock_values["BYPASS_ME"],
  }

  # calculate and insert basic checksum
  dat = packer.make_can_msg("STEER_CMD", 0, values)[1]
  values["BASIC_CHECKSUM"] = gwm_basic_chksum_for_0x12B(dat)
  # calculate and insert CRC
  dat = packer.make_can_msg("STEER_CMD", 0, values)[1]
  values["CRC_X9B"] = checksum(dat[9:16], 0x9B)

  return packer.make_can_msg("STEER_CMD", CAN.main, values)


def create_longitudinal_command(packer, CAN, longitudinal_stock_values, accel, active, standstill):
  values = {s: longitudinal_stock_values[s] for s in [
    "BYPASSME_1",
    "SPEED_REAL",
    "COUNTER_BRAKE",
    "BYPASSME_2",
    "BYPASS_ACC1",
    "BYPASS_ACC2",
    "COUNTER_ACC",
  ]}

  brake_or_gas = longitudinal_stock_values["BRAKE_OR_GAS_REQ"]
  standstill1 = longitudinal_stock_values["STANDSTILL_1"]
  standstill2 = longitudinal_stock_values["STANDSTILL_2"]
  standstill3 = longitudinal_stock_values["STANDSTILL_3"]
  brake_cmd = 0
  accel_cmd = 0
  if accel < 0 and active:
    brake_or_gas = 13
    brake_cmd = (accel * (107 - 41)) - 41
    accel_cmd = 0
    standstill1 = 1 if standstill else 0
    standstill2 = 3 if standstill else 4 # 3 "active" 4 "inactive"
    standstill3 = 0 if standstill else 1 # 0 "active" 1 "inactive"
  elif active:
    brake_or_gas = 12
    brake_cmd = 0
    # accel_cmd = accel * 4577
    accel_cmd = np.interp(accel, [0.25, 1], [0, 4577])
    standstill1 = 0
    standstill2 = 4 # 3 "active" 4 "inactive"
    standstill3 = 1 # 0 "active" 1 "inactive"
  values |= {
    "BRAKE_OR_GAS_REQ": brake_or_gas,
    "BRAKE_CMD": brake_cmd,
    "GAS_CMD": accel_cmd,
    "STANDSTILL_1": standstill1,
    "STANDSTILL_2": standstill2,
    "STANDSTILL_3": standstill3,
  }

  data = packer.make_can_msg("ACC_CMD", 0, values)[1]
  values["CRC_BRAKE_0xEF"] = checksum(data[9:16], 0xEF)
  values["CRC_ACC_0x87"] = checksum(data[25:32], 0x87)

  return packer.make_can_msg("ACC_CMD", CAN.main, values)


def create_wheel_touch(packer, CAN: CanBus, eps_stock_values, ea_simulated_torque: float):
  values = {s: eps_stock_values[s] for s in [
    "A_CRC_X61",
    "A_BYPASSME_2",
    "A_RX_STEER_REQUESTED",
    "A_BYPASSME_1",
    "A_COUNTER",
    "B_CRC_X61",
    "B_RX_DRIVER_TORQUE",
    "B_BYPASSME_1",
    "B_BYPASSME_2",
    "B_RX_EPS_TORQUE",
    "B_COUNTER",
    "B_BYPASSME_3",
  ]}

  values.update({
    "B_RX_DRIVER_TORQUE": ea_simulated_torque,
  })

  # calculate checksum
  dat = packer.make_can_msg("RX_STEER_RELATED", 0, values)[1]
  values["B_CRC_X61"] = checksum(dat[9:16], 0x61)

  return packer.make_can_msg("RX_STEER_RELATED", CAN.camera, values)


def create_buttons_command(packer, CAN: CanBus, counter, stock_msg, cancel_command=False):
  values = {s: stock_msg[s] for s in [
    "STEERING_ANGLE",
    "STEERING_DIRECTION",
    "STEERING_RATE",
    "RATE_DIRECTION",
    "AP_ENABLE_COMMAND",
    "AP_DECREASE_SPEED_COMMAND",
    "AP_INCREASE_SPEED_COMMAND",
  ]}

  values |= {
    "AP_REDUCE_DISTANCE_COMMAND": 0,
    "AP_INCREASE_DISTANCE_COMMAND": 0,
    "AP_CANCEL_COMMAND":  stock_msg["AP_CANCEL_COMMAND"] or cancel_command,
    "COUNTER": counter,
  }

  data = packer.make_can_msg("STEER_AND_AP_STALK", 0, values)[1]
  values["CRC_X2D"] = checksum(data[1:8], 0x2D)

  return packer.make_can_msg('STEER_AND_AP_STALK', CAN.camera, values)


def checksum(data, xor_output):
  crc = 0
  poly = 0x1D
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ poly) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ xor_output


def gwm_basic_chksum_for_0x12B(d: bytearray) -> int:
  invert_direction = d[12] >> 7 & 0x1
  counter = d[15] & 0xF
  steer_requested = d[15] >> 5 & 0x1
  return (28 - (steer_requested * 8) - counter - invert_direction) & 0x1F
