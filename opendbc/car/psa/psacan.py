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
  checksum = sum((b >> 4) + (b & 0xF) for b in dat)
  return (11 - checksum) & 0xF

def get_status(frame: int, lat_active: bool, _state={"activation_frame": 0, "lat_active_prev": False}) -> int:
    if not _state["lat_active_prev"] and lat_active:
        _state["activation_frame"] = frame
    _state["lat_active_prev"] = lat_active
    if not lat_active:
        return 2
    return 2 + min((frame - _state["activation_frame"]), 2)

def create_lka_msg(packer, CP, apply_angle: float, frame: int, lat_active: bool, ramp_value: int, steeringPressed: bool):
  values = {
    'DRIVE': 1,
    'COUNTER': (frame // 5) % 0x10,
    'CHECKSUM': 0,
    'STATUS': get_status(frame, lat_active), # 2: READY, 3: AUTHORIZED, 4: ACTIVE
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1]
  if isinstance(msg, int):
    msg = msg.to_bytes(1, 'big')
  values['CHECKSUM'] = calculate_checksum(msg)

  # TODO: swap CAN 0/2 on harness
  return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).camera, values)
