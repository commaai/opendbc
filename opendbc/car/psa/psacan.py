def psa_checksum(address: int, sig, d: bytearray) -> int:
  if address == 0x452:
    chk_ini = 0x4
  elif address == 0x38D:
    chk_ini = 0x7
  else:
    chk_ini = 0xB
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
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

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
