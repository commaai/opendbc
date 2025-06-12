def calculate_checksum(dat: bytearray, chk_ini: int) -> int:
  checksum = sum((b >> 4) + (b & 0xF) for b in dat)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, frame: int, lat_active: bool, apply_angle: float):
  values = {
    'DRIVE': 1,
    'COUNTER': frame % 0x10,
    'CHECKSUM': 0,
    # Cycle STATUS 2->3->4->2.. this keeps control active. 0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
    'STATUS': (frame % 3) + 2 if lat_active else 0,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1]
  values['CHECKSUM'] = calculate_checksum(msg, 0xB)

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
