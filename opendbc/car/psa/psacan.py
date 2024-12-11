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


def calculate_checksum(dat: bytearray, init: int) -> int:
  checksum = init
  for i in range(len(dat)):
    # assumes checksum is zeroed out
    checksum += (dat[i] & 0xF) + (dat[i] >> 4)
  return (8 - checksum) & 0xF


def create_lka_msg(packer, CP, apply_steer: float, frame: int, lat_active: bool, max_torque: int):
  # TODO: hud control for lane departure, status
  values = {
    'TORQUE': max_torque if lat_active else 0,
    'LANE_DEPARTURE': 0,
    'STATUS': 2 if lat_active else 0,
    'RAMP': 100 if lat_active else 0,  # TODO
    'ANGLE': clip(apply_steer, -90, 90),  # (-90, 90)
    'COUNTER': frame % 0x10,
    'CHECKSUM': 0,
  }

  # calculate checksum
  dat = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[2]
  values['CHECKSUM'] = calculate_checksum(dat, 0xD)

  return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).main, values)
