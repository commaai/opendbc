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


# TODO: add checksum / CRC handlers
