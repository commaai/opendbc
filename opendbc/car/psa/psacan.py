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

def create_lka_msg(packer, CP, apply_steer: float, frame: int, lat_active: bool, max_torque: int, reverse: bool):
    # TODO: hud control for lane departure, status
    # TODO: unknowns could be recuperation mode/drive mode
    values = {
        'unknown1': 0 if reverse else 1,
        'COUNTER': (frame//5) % 0x10,
        'CHECKSUM': 0,
        'unknown2': 0x0C, #TODO: analyze original signal
        'TORQUE': max_torque if lat_active else 0,
        'LANE_DEPARTURE': 2 if max_torque > 0 else 1 if max_torque < 0 else 0, # TODO: check sign
        'LKA_DENY': 0,
        'STATUS': 2 if lat_active else 0,
        'unknown3': 1,
        'RAMP': 100 if lat_active else 0, # TODO maybe implement ramping
        'ANGLE': clip(apply_steer, -90, 90),
        'unknown4': 1,
    }

    msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
    dat = msg[1]
    if isinstance(dat, int):
        dat = dat.to_bytes(1, 'big')

    values['CHECKSUM'] = calculate_checksum(dat)

    return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).main, values)
