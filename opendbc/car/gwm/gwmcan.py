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


# CRC-8 0x2F OEM-style for GWM 0x12B message
def gwm_crc8_2f_for_0x12B(address: int, sig, d: bytearray) -> int:
  crc = 0xFF
  poly = 0x2F
  # bytes 9..14
  for byte in d[9:15]:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  # byte 15: only data bits 4..7
  b15 = d[15] & 0xF0
  crc ^= b15
  for _ in range(8):
    if crc & 0x80:
      crc = ((crc << 1) ^ poly) & 0xFF
    else:
      crc = (crc << 1) & 0xFF
  return crc


def create_steer_and_ap_stalk(packer, steer_msg, fake_torque=False, bus=0):
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
  return packer.make_can_msg('STEER_AND_AP_STALK', bus, values)
