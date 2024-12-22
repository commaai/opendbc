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

# TODO: remove original values
def create_lka_msg(packer, CP, apply_angle: float, frame: int, lat_active: bool, max_torque: int, ramp_value: int, driving: bool):
    # Construct message
    values = {
        'unknown1': 1 if driving else 0, # TODO: rename to DRIVING
        'COUNTER': (frame // 5) % 0x10,
        'CHECKSUM': 0,
        'unknown2': 0x8B,# original_lkas_values['unknown2'], # TODO: check if forward is ok, currently ramps up 1/s up to 0x0B
        'TORQUE': 2043 if lat_active else 0, # TODO: maybe better to do with apply-angle_active or something. oscillates around 2043
        'LANE_DEPARTURE': 0, # not used in HDA # 2 if apply_steer < 0 else 1 if apply_steer > 0 else 0,
        'STATUS': 4 if lat_active else 2,
        'LXA_ACTIVATION': 0,
        'TORQUE_FACTOR': ramp_value, #TODO: rename to torque_factor
        'SET_ANGLE': apply_angle, #TODO: rename dbc to APPLY_ANGLE
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
