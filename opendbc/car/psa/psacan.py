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
    'DMD_DELEST_DA': 0, # 0: Not Required, 1: Power Cut Request
    'UNKNOWN_1': 1, # unknown, 0 - 1, set to 1
    'COUNTER': frame % 0x10, # 0 - 15, counter for the message
    'CHECKSUM': 0, # 0 - 15, checksum for the message
    'UNKNOWN_2': 0, # 0 - 15, set to 0
    'UNKNOWN_3': 0, # 0 - 1, 1 for first 3 min of drive, then 0
    'LKA_STATE': 4 if lat_active else 3, # 0: Unavailable, 1: Unselected, 2: Selected, 3: Authorized
                                         # 4: Active, 5: Defect, 6: Collision, 7: Reserved
    'LXA_ACTIVATION': 1, # 0: LKA Function (Lane Keep Assist), 1: LPA Function (Lane Position Assist)
    'LKA_TRQ_FACT_REQ': 0, # 0 - 1, Control authority of the EPS, needs to be ramped up/down when LKA_STATE is active
    'COLUMN_ANGLE_SETPOINT': clip(apply_steer, -90, 90),  # (-90, 90), angle based steering
  }

# TODO: fix checksum
  # calculate checksum
  # dat = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[2]
  # values['CHECKSUM'] = calculate_checksum(dat, 0xD)

  return 0#packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).main, values)
