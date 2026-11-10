def leapmotor_checksum(address: int, sig, d: bytearray) -> int:
  return (0xFF - sum(d[:7])) & 0xFF


def create_lka_command(packer, lka_state: int, torque: int, side: int, ldw: bool, counter: int):
  values = {
    "LKA_STATE": lka_state,
    "STEER_ANGLE_REQ": 0,
    "LKA_TORQUE_REQ": torque,
    "LKA_SIDE": side,
    "LDW_ACTIVE": ldw,
    "COUNTER": counter,
  }
  return packer.make_can_msg("EPS_LKA_CMD", 0, values)
