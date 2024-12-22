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
    # find CHK so that (checksum + CHK) % 16 = 11 (0xB)
    needed = (11 - checksum) & 0xF
    return needed

def create_lka_msg(packer, CP, apply_angle: float, frame: int, lat_active: bool, max_torque: int, ramp_value: int, driving: bool):
    # Used for status phase change 2->3->4->3->2
    phase = (frame % 10) // 5
    # Construct message
    values = {
        'unknown1': 1 if driving else 0, # TODO: check if required
        'COUNTER': (frame // 5) % 0x10, # REQUIRED
        'CHECKSUM': 0, # REQUIRED
        'unknown2': 0x0B, #TODO: check if required
        'TORQUE': 0, # check if required
        'LANE_DEPARTURE': 0, # check if required
        'STATUS': 2 if lat_active and ((frame % 15) // 5) == 0 else 3 if ((frame % 15) // 5) == 1 else 4 if lat_active else 4 if not lat_active and ((frame % 15) // 5) == 2 else 3 if ((frame % 15) // 5) == 1 else 2, # REQUIRED
        'LXA_ACTIVATION': lat_active, # REQUIRED
        'TORQUE_FACTOR': ramp_value, # REQUIRED
        'SET_ANGLE': apply_angle, # TODO: rename dbc to APPLY_ANGLE
        'unknown4': 1, # check if required
    }

    msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
    dat = msg[1]
    if isinstance(dat, int):
        dat = dat.to_bytes(1, 'big')
    values['CHECKSUM'] = calculate_checksum(dat)

    return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).camera, values)
