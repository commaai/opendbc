def lka_command_torque(lka_command_2_torque):
  # LKA_COMMAND_2 / 4, rounded half away from zero
  magnitude = (abs(lka_command_2_torque) + 2) // 4
  return magnitude if lka_command_2_torque >= 0 else -magnitude


def create_steering_control(packer, bus, msg, apply_steer, lat_active):
  values = {
    "LKA_ACTIVE": lat_active,
    "LKA_TORQUE": apply_steer,
  }

  return packer.make_can_msg(msg, bus, values)


def create_lka_hud_2_control(packer, bus, lat_active):
  # Cluster lane display. Stock values (NEW_SIGNAL_1, LKA_ACTIVE) seen with the dash video:
  #   (6, 0) white lines, green curve: lane assist ready
  #   (1, 0) white lines: lane assist not ready
  #   (2, 1), (4, 1) right line yellow; (0, 1), (3, 1) left line yellow: correcting a lane departure
  # Stock sets LKA_ACTIVE only during a short correction. Holding it for a whole engagement showed a
  # yellow line and curve, and is suspected of the "Stay in Lane" nag. It doesn't affect the EPS.
  values = {
    "LKA_DISABLE": 0,
    "NEW_SIGNAL_1": 6 if lat_active else 1,
    "LKA_ACTIVE": 0,
  }

  return packer.make_can_msg("LKA_HUD_2", bus, values)


def create_lka_hud_3_control(packer, bus, lat_active):
  # NEW_SIGNAL_1 omitted -- may be haptic steering wheel feedback level, probably not consequential to steering control
  values = {
    "LKA_ACTIVE": lat_active,
  }

  return packer.make_can_msg("LKA_HUD_3", bus, values)


def create_acc_button_control(packer, bus, counter, cancel=False):
  values = {
    "CANCEL": cancel,
    "COUNTER": counter % 0x10,
    "UNKNOWN_1": 3,
    "UNKNOWN_2": 0x0E,
  }

  return packer.make_can_msg("ACC_BUTTON", bus, values)
