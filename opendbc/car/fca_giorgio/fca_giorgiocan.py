def create_steering_control(packer, bus, msg, apply_steer, lat_active):
  values = {
    "LKA_ACTIVE": lat_active,
    "LKA_TORQUE": apply_steer,
  }

  return packer.make_can_msg(msg, bus, values)


def create_lka_hud_2_control(packer, bus, lat_active):
  values = {
    "LKA_DISABLE": 0,
    # roughly: bit 1 = lane detection on, bit 2 = right line detected, bit 4 = left line detected
    "NEW_SIGNAL_1": 6,
    "LKA_ACTIVE": lat_active,
  }

  return packer.make_can_msg("LKA_HUD_2", bus, values)


def create_lka_hud_3_control(packer, bus, lat_active):
  # NEW_SIGNAL_1 omitted -- may be haptic steering wheel feedback level, probably not consequential to steering control
  values = {
    "LKA_ACTIVE": lat_active,
  }

  return packer.make_can_msg("LKA_HUD_3", bus, values)
