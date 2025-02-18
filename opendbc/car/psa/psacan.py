from opendbc.car import CanBusBase

class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset + 2

  @property
  def adas(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset

def calculate_checksum(dat: bytearray) -> int:
  checksum = sum((b >> 4) + (b & 0xF) for b in dat)
  return (11 - checksum) & 0xF

def create_lka_msg(packer, CP, frame: int, lat_active: bool, apply_angle: float):
  values = {
    'DRIVE': 1,
    'COUNTER': (frame // 5) % 0x10,
    'CHECKSUM': 0,
    'STATUS': (frame % 3) + 2 if lat_active else 2, # Cycle status 2->3->4->2.. this keeps control active 2: READY, 3: AUTHORIZED, 4: ACTIVE
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1]
  if isinstance(msg, int):
    msg = msg.to_bytes(1, 'big')
  values['CHECKSUM'] = calculate_checksum(msg)

  return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).camera, values)
