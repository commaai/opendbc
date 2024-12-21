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

def create_lka_msg_only_chks(packer, CP, original_lka_values):
    # values = original_lka_values.copy()
    # values['CHECKSUM'] = 0
    # msg = packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).main, values)
    # dat = msg[1]
    # if isinstance(dat, int):
    #     dat = dat.to_bytes(1, 'big')
    # values['CHECKSUM'] = calculate_checksum(dat)
    return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).camera, original_lka_values)

def create_lka_msg(packer, CP, apply_steer: float, steering_angle: float, frame: int, lat_active: bool, max_torque: int, ramp_value: int, reverse: bool):

    # Construct message
    values = {
        'unknown1': 0 if reverse else 1, # TODO: rename to REVERSE
        'COUNTER': (frame // 5) % 0x10,
        'CHECKSUM': 0,
        'unknown2': 0x0B, # TODO: check, currently ramps up 1/s up to 0x0B
        'TORQUE': apply_steer,
        'LANE_DEPARTURE': 2 if apply_steer < 0 else 1 if apply_steer > 0 else 0,
        'LKA_DENY': 0 if apply_steer != 0 else 1,
        'STATUS': 2 if apply_steer != 0 else 1,
        'unknown3': 0,
        'RAMP': ramp_value,
        'ANGLE': steering_angle,
        'unknown4': 1,
    }

    print(f"Message values: {values}")

    msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
    dat = msg[1]
    if isinstance(dat, int):
        dat = dat.to_bytes(1, 'big')

    # Compute and log checksum
    values['CHECKSUM'] = calculate_checksum(dat)
    print(f"Final CHECKSUM: {values['CHECKSUM']}")

    return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).camera, values)
