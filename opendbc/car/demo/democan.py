def create_steering_control(packer, bus, apply_torque, lkas_enabled):
  values = {
    # "LKAS_ENABLED": lkas_enabled,
    # "LKAS_TORQUE": apply_torque,
  }
  return packer.make_can_msg("LKAS_CONTROL", bus, values)


def create_lka_hud_control(packer, bus, lat_active, hud_control):
  values = {
    # "LKAS_ENABLED": lat_active,
    # "LANE_LINE_LEFT": hud_control.leftLaneVisible,
    # "LANE_LINE_RIGHT": hud_control.rightLaneVisible,
  }
  return packer.make_can_msg("LKAS_HUD", bus, values)


def create_acc_buttons_control(packer, bus, counter, cancel=False, resume=False):
  values = {
    # "COUNTER": (counter + 1) % 16,
    # "BUTTON_CANCEL": cancel,
    # "BUTTON_RESUME": resume,
  }
  return packer.make_can_msg("ACC_BUTTONS", bus, values)


def demo_checksum(address: int, sig, d: bytearray) -> int:
  """

  Placeholder checksum function: implements a simple XOR over the payload, including the counter. Newer cars generally
  use a CRC algorithm for which the parameters need to be identified. Tools that may help you find the right solution:

  * https://github.com/colinoflynn/crcbeagle
  * https://github.com/resilar/crchack

  Also search for "AUTOSAR E2E Protocol Specification" and consider the algorithms there, VW/Audi use E2E Profile 2

  For openpilot to calculate and verify any payload protection algorithm, the signal MUST be named exactly "CHECKSUM",
  no matter what algorithm is used. Likewise, counters must be named "COUNTER" for openpilot to verify and increment.

  You MUST update the DEMO_CHECKSUM case of get_checksum_state() in opendbc/can/dbc.py to reflect the correct bit-width
  of your checksum and counter signals.

  """
  checksum = 0
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum ^= d[i]
  return checksum
