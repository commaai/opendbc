from opendbc.car.common.numpy_fast import clip
from opendbc.car import CanBusBase

class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def adas(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2

def calculate_checksum(dat: bytearray) -> int:
    checksum = 0
    for i, b in enumerate(dat):
        high_nibble = b >> 4
        low_nibble = b & 0xF
        checksum += high_nibble + low_nibble
    needed = (11 - checksum) & 0xF
    return needed

def create_lka_msg(packer, CP, apply_angle: float, frame: int, lat_active: bool, ramp_value: int):
    values = {
        'COUNTER': (frame // 5) % 0x10,
        'CHECKSUM': 0,
        'STATUS': ((frame // 5) % 3) + 2, # ramp status 2->3->4->2->3->4...
        'LXA_ACTIVATION': lat_active,
        'TORQUE_FACTOR': ramp_value,
        'SET_ANGLE': apply_angle,
    }

    msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1]
    if isinstance(msg, int):
        msg = msg.to_bytes(1, 'big')
    values['CHECKSUM'] = calculate_checksum(msg)

    return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).camera, values)
