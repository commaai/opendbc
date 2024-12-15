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
    # sum all nibbles, with the checksum nibble set to 0
    checksum = 0
    for b in dat:
        checksum += (b >> 4) + (b & 0xF)
    # find CHK so that (checksum + CHK) % 16 = 11 (0xB)
    return (11 - checksum) & 0xF

def create_lka_msg(packer, CP, apply_steer: float, frame: int, lat_active: bool, max_torque: int):
  # TODO: hud control for lane departure, status
  # TODO: unknowns could be recuperation mode/drive mode
  values = {
    'unknown1': 1,
    'COUNTER': frame % 0x50, #TODO: find factor
    'CHECKSUM': 0,
    'unknown2': 0x0C, #TODO: lsb changes sometimes
    'TORQUE': max_torque if lat_active else 0,
    'LANE_DEPARTURE': 2 if apply_steer > 0 else 1 if apply_steer < 0 else 0,
    'LKA_DENY': 0,
    'STATUS': 2 if lat_active else 0,
    'unknown4': 1,
    'RAMP': 100 if lat_active else 0,  # TODO maybe implement ramping
    'ANGLE': clip(apply_steer, -90, 90),  # (-90, 90)
    'unknown4': 1,
  }

  dat = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[2]
  # make tests pass
  if isinstance(dat, int):
      dat = dat.to_bytes(1, 'big')

  # calculate checksum
  values['CHECKSUM'] = calculate_checksum(dat)

  return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).main, values)