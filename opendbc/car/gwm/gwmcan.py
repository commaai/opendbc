def gwm_checksum(address: int, sig, d: bytearray) -> int:
    crc = 0x00
    poly = 0x1D
    xor_out = 0x2D
    for byte in d:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc = (crc << 1)
            crc &= 0xFF
    return crc ^ xor_out

def create_lka_steering(packer, lat_active: bool, apply_angle: float, status: int):
  values = {
    'DRIVE': 1,
    'STATUS': status,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
