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


def create_eps_update(packer, CAN: CanBus, eps_stock_values, ea_simulated_torque: float):
  values = {s: eps_stock_values[s] for s in [
    "A_CRC_X61",
    "A_RX_STEER_REQUESTED",
    "A_SET_ME_X50",
    "A_SET_ME_X01",
    "A_COUNTER",
    "B_CRC_X61",
    "B_SET_ME_X01",
    "B_SET_ME_X0C",
    "B_SET_ME_X05",
    "B_SET_ME_XDC",
    "B_RX_EPS_ANGLE",
    "B_SET_ME__X01",
    "B_BYPASS_ME",
    "B_SET_ME_X03",
  ]}

  values.update({
    "B_RX_DRIVER_TORQUE": ea_simulated_torque,
    "B_COUNTER": (eps_stock_values["B_COUNTER"] + 1) % 16,
  })

  # calculate checksum
  dat = packer.make_can_msg("RX_STEER_RELATED", 0, values)[1]
  values["B_CRC_X61"] = gwm_crc_for_0x147(0, 0, dat)

  return packer.make_can_msg("RX_STEER_RELATED", CAN.camera, values)


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
