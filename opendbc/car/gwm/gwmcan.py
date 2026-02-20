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
  values["BASIC_CHECKSUM"] = gwm_basic_chksum_for_0x12B(0, 0, dat)
  # calculate and insert CRC
  dat = packer.make_can_msg("STEER_CMD", 0, values)[1]
  values["CRC_X9B"] = gwm_crc_for_0x12B(0, 0, dat)

  return packer.make_can_msg("STEER_CMD", CAN.main, values)


def create_eps_update(packer, CAN: CanBus, eps_stock_values, ea_simulated_torque: float):
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
  values["B_CRC_X61"] = gwm_crc_for_0x147(0, 0, dat)

  return packer.make_can_msg("RX_STEER_RELATED", CAN.camera, values)


def bypass_steer_cmd(packer, CAN: CanBus, camera_stock_values):
  values = {s: camera_stock_values[s] for s in [
    "CRC_X9B",
    "TORQUE_REFLECTED",
    "BASIC_CHECKSUM",
    "SET_ME_X01",
    "TORQUE_CMD",
    "INVERT_DIRECTION",
    "COUNTER",
    "STEER_REQUEST",
    "BYPASS_ME",
  ]}

  return packer.make_can_msg("STEER_CMD", CAN.main, values)


def gwm_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0x00
  poly = 0x1D
  xor_out = 0x2D
  for byte in d[1:]:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = (crc << 1) ^ poly
      else:
        crc = (crc << 1)
      crc &= 0xFF
  return crc ^ xor_out


def gwm_crc_for_0x12B(address: int, sig, d: bytearray) -> int:
  crc = 0x00
  poly = 0x1D
  xor_out = 0x9B
  for byte in d[9:16]:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ poly) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ xor_out


def gwm_basic_chksum_for_0x12B(address: int, sig, d: bytearray) -> int:
  invert_direction = d[12] >> 7 & 0x1
  counter = d[15] & 0xF
  steer_requested = d[15] >> 5 & 0x1
  return (28 - (steer_requested * 8) - counter - invert_direction) & 0x1F


def gwm_crc_for_0x147(address: int, sig, d: bytearray) -> int:
  crc = 0x00
  poly = 0x1D
  xor_out = 0x61
  for byte in d[9:16]:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ poly) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ xor_out


def create_steer_and_ap_stalk(packer, CAN: CanBus, steer_msg, fake_torque=False):
  """
  Copy STEER_AND_AP_STALK message from bus 0 and forward to bus 2,
  copying all signals unchanged. If the DBC generator renames the checksum
  signal to `_CHECKSUM`, prefer copying `_CHECKSUM` so the packer does not
  recompute it.
  """
  # Prefer renamed checksum field `_CHECKSUM` if present, otherwise don't
  # include CHECKSUM so packer may compute it if needed.
  values = {}
  if '_CHECKSUM' in steer_msg:
    values['_CHECKSUM'] = steer_msg['_CHECKSUM']

  # Copy all native signals unchanged
  values.update({
    'STEERING_ANGLE': steer_msg['STEERING_ANGLE'],
    'STEERING_DIRECTION': steer_msg['STEERING_DIRECTION'],
    'STEERING_TORQUE': steer_msg['STEERING_TORQUE'],
    'EPS_ACTUATING': steer_msg['EPS_ACTUATING'],
    'AP_REDUCE_DISTANCE_COMMAND': steer_msg['AP_REDUCE_DISTANCE_COMMAND'],
    'AP_INCREASE_DISTANCE_COMMAND': steer_msg['AP_INCREASE_DISTANCE_COMMAND'],
    'AP_CANCEL_COMMAND': steer_msg['AP_CANCEL_COMMAND'],
    'AP_ENABLE_COMMAND': steer_msg['AP_ENABLE_COMMAND'] or fake_torque,
    'AP_DECREASE_SPEED_COMMAND': steer_msg['AP_DECREASE_SPEED_COMMAND'],
    'AP_INCREASE_SPEED_COMMAND': steer_msg['AP_INCREASE_SPEED_COMMAND'],
    'COUNTER': (steer_msg['COUNTER'] + 1) % 16,
  })
  return packer.make_can_msg('STEER_AND_AP_STALK', CAN.camera, values)
