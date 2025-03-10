from opendbc.car import CanBusBase

class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset + 2

  @property
  def adas(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset

def calculate_checksum(dat: bytearray) -> int:
  checksum = sum((b >> 4) + (b & 0xF) for b in dat)
  return (11 - checksum) & 0xF

def create_lka_msg(packer, CP, frame: int, lat_active: bool, apply_angle: float):
  values = {
    'DRIVE': 1,
    'COUNTER': (frame // 5) % 0x10,
    'CHECKSUM': 0,
    'STATUS': (frame % 3) + 2 if lat_active else 2, # Cycle status 2->3->4->2.. this keeps control active 2: READY, 3: AUTHORIZED, 4: ACTIVE
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1]
  if isinstance(msg, int):
    msg = msg.to_bytes(1, 'big')
  values['CHECKSUM'] = calculate_checksum(msg)

  return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).camera, values)


def create_cancel_request(packer, CP, frame: int, set_speed: int):
  values = {
    'LONGITUDINAL_REGULATION_TYPE': 3,
    'TURN_SIGNAL_STATUS': 0,
    'FRONT_WIPER_STATUS': 0,
    'VEHICLE_SPEED_LIMIT_SETPOINT': set_speed,
    'CHECKSUM_CONS_RVV_LVV2': (((set_speed >> 4) & 1) << 1) | (set_speed & 1),
    'BRAKE_ONLY_CMD_BSI': 0,
    'LVV_ACTIVATION_REQ': 0,
    'RVV_ACC_ACTIVATION_REQ': 0,
    'ARC_HABIT_SENSITIVITY': 2, # TODO: check
    'ARC_HABIT_ACTIVATION_REQ': 0,
    'FRAME_COUNTER_BSI2': (frame // 5) % 0x10,
    'FRONT_WASH_STATUS': 0,
    'FORCE_ACTIVATION_HAB_CMD': 1, # TODO: check
    'INTER_VEHICLE_TIME_SETPOINT': 6.2, # TODO: check
    'CHECKSUM_FRAME_4B_452': 0,
    'COCKPIT_GO_ACC_REQUEST': 0,
    'ACC_PROGRAM_MODE': 0,
  }

  msg = packer.make_can_msg('HS2_DAT_MDD_CMD_452', 0, values)[1]
  if isinstance(msg, int):
    msg = msg.to_bytes(1, 'big')

  values['CHECKSUM_FRAME_4B_452'] = calculate_checksum(msg)

  return packer.make_can_msg('HS2_DAT_MDD_CMD_452', CanBus(CP).adas, values)
