import opendbc.car.volkswagen.mqbcan as mqbcan


create_eps_update = mqbcan.create_eps_update
create_lka_hud_control = mqbcan.create_lka_hud_control
create_acc_buttons_control = mqbcan.create_acc_buttons_control


def create_steering_control(packer, bus, apply_curvature, lkas_enabled, power):
  values = {
    "Curvature": abs(apply_curvature), # in rad/m
    "Curvature_VZ": 1 if apply_curvature > 0 and lkas_enabled else 0,
    "Power": power if lkas_enabled else 0,
    "RequestStatus": 4 if lkas_enabled else 2,
    "HighSendRate": lkas_enabled,
  }
  return packer.make_can_msg("HCA_03", bus, values)


def create_ea_control(packer, bus):
  values = {
    "EA_Funktionsstatus": 1,  # Configured but disabled
    "EA_Sollbeschleunigung": 2046,  # Inactive value
  }

  return packer.make_can_msg("EA_01", bus, values)


def create_ea_hud(packer, bus):
  values = {
    "EA_Unknown": 1,  # Undocumented, value when inactive
  }

  return packer.make_can_msg("EA_02", bus, values)
