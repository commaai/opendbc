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
    """Calculate nibble-based checksum and add debug prints."""
    print("DEBUG: Starting checksum calculation")
    print(f"DEBUG: Input data (hex): {dat.hex()}")

    checksum = 0
    for i, b in enumerate(dat):
        high_nibble = b >> 4
        low_nibble = b & 0xF
        checksum += high_nibble + low_nibble
        print(f"DEBUG: Byte [{i}]: 0x{b:02X}, high_nibble=0x{high_nibble:X}, low_nibble=0x{low_nibble:X}, "
              f"partial_checksum={checksum}")

    # find CHK so that (checksum + CHK) % 16 = 11 (0xB)
    needed = (11 - checksum) & 0xF
    print(f"DEBUG: final nibble-sum={checksum}, needed nibble to get 0xB={needed}")
    return needed

def create_lka_msg(packer, CP, apply_steer: float, frame: int, lat_active: bool, max_torque: int):
    values = {
        'unknown1': 1,
        'COUNTER': (frame//5) % 0x10,
        'CHECKSUM': 0,  # will be overwritten
        'unknown2': 0x0C,
        'TORQUE': max_torque if lat_active else 0,
        'LANE_DEPARTURE': 2 if apply_steer > 0 else 1 if apply_steer < 0 else 0,
        'LKA_DENY': 0,
        'STATUS': 2 if lat_active else 0,
        'unknown3': 1,
        'RAMP': 100 if lat_active else 0,
        'ANGLE': clip(apply_steer, -90, 90),
        'unknown4': 1,
    }

    # First build to see raw data (Use index [1] for data, not [2])
    raw_msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
    dat = raw_msg[1]
    if isinstance(dat, int):
        dat = dat.to_bytes(1, 'big')

    print("DEBUG: Values before checksum calculation:")
    for k, v in values.items():
        print(f"  {k}: {v}")
    print("DEBUG: Raw data before checksum:")
    print(f"  {dat.hex()}")

    # Calculate checksum
    chksum = calculate_checksum(dat)
    values['CHECKSUM'] = chksum

    print(f"DEBUG: Applying checksum {chksum} back to message")

    # Build final message with updated checksum (again, data is at index [1])
    final_msg = packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).main, values)
    final_dat = final_msg[1]
    if isinstance(final_dat, int):
        final_dat = final_dat.to_bytes(1, 'big')

    print("DEBUG: Final values after checksum calculation:")
    for k, v in values.items():
        print(f"  {k}: {v}")
    print("DEBUG: Final message data:")
    print(f"  {final_dat.hex()}")
    print("DEBUG: End of create_lka_msg\n")

    return final_msg
