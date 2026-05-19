def create_steering_control(packer, bus, apply_curvature, lkas_enabled, power=0):
  values = {
    "Curvature": abs(apply_curvature),  # in rad/m
    "Curvature_VZ": 1 if apply_curvature > 0 and lkas_enabled else 0,
    "Power": power if lkas_enabled else 0,
    "RequestStatus": 4 if lkas_enabled else 2,
    "HighSendRate": lkas_enabled,
  }
  return packer.make_can_msg("HCA_03", bus, values)


def create_eps_update(packer, bus, eps_stock_values, ea_simulated_torque):
  values = {s: eps_stock_values[s] for s in [
    "COUNTER",                     # Sync counter value to EPS output
    "EPS_Lenkungstyp",             # EPS rack type
    "EPS_Berechneter_LW",          # Absolute raw steering angle
    "EPS_VZ_BLW",                  # Raw steering angle sign
    "EPS_HCA_Status",              # EPS HCA control status
  ]}

  values.update({
    # Absolute driver torque input and sign, with EA inactivity mitigation
    "EPS_Lenkmoment": abs(ea_simulated_torque),
    "EPS_VZ_Lenkmoment": 1 if ea_simulated_torque < 0 else 0,
  })

  return packer.make_can_msg("LH_EPS_03", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control, sound_alert=False):
  display_mode = 1 if lat_active else 0  # travel assist style showing yellow lanes when op is active

  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",   # Blind spot in warning mode on left side due to lane departure
      "LDW_SW_Warnung_rechts",  # Blind spot in warning mode on right side due to lane departure
      "LDW_Seite_DLCTLC",       # Direction of most likely lane departure (left or right)
      "LDW_DLC",                # Lane departure, distance to line crossing
      "LDW_TLC",                # Lane departure, time to line crossing
    ]}

  values.update({
    "LDW_Gong": sound_alert,
    "LDW_Status_LED_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 + display_mode if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible + display_mode,
    "LDW_Lernmodus_rechts": 3 + display_mode if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible + display_mode,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False, up=False, down=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalter",           # ACC button, on/off
    "GRA_Typ_Hauptschalter",       # ACC main button type
    "GRA_Codierung",               # ACC button configuration/coding
    "GRA_Tip_Stufe_2",             # unknown related to stalk type
    "GRA_ButtonTypeInfo",          # unknown related to stalk type
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Tip_Wiederaufnahme": resume or up,
    "GRA_Tip_Setzen": down,
  })
  return packer.make_can_msg("GRA_ACC_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  commands = []

  acceleration = accel if acc_enabled else 3.01

  if starting:
    acc_hold_type = 4  # release request and startup
  elif stopping and esp_hold:
    acc_hold_type = 3  # hold standby
  elif stopping:
    acc_hold_type = 1  # hold or hold request
  else:
    acc_hold_type = 0  # no hold request

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_control,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  acceleration,
    "ACC_zul_Regelabw_unten":     0.2 if acc_enabled else 0,
    "ACC_zul_Regelabw_oben":      0.2 if acc_enabled else 0,
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,
    "ACC_Anfahren":               starting,
    "ACC_Anhalten":               1 if stopping and not esp_hold else 0,
    "ACC_Anhalteweg":             0 if stopping and not esp_hold else 20.46,
    "ACC_Anforderung_HMS":        acc_hold_type,
    "ACC_AKTIV_regelt":           1 if acc_control == 3 else 0,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }

  commands.append(packer.make_can_msg("ACC_18", bus, values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  values = {
    "ACC_Status_ACC":          acc_hud_status,
    "ACC_Wunschgeschw_02":     set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": distance + 2,
    "ACC_Display_Prio":        3,
    "ACC_Abstandsindex_02":    569 if lead_distance else 0,
  }

  return packer.make_can_msg("MEB_ACC_01", bus, values)


def create_capacitive_wheel_touch(packer, bus, lat_active, klr_stock_values):
  values = {s: klr_stock_values[s] for s in [
    "COUNTER",
    "KLR_Touchintensitaet_1",
    "KLR_Touchintensitaet_2",
    "KLR_Touchintensitaet_3",
    "KLR_Touchauswertung",
  ]}

  if lat_active:
    values.update({
      "COUNTER": (klr_stock_values["COUNTER"] + 1) % 16,
      "KLR_Touchintensitaet_1": 80,
      "KLR_Touchintensitaet_2": 200,
      "KLR_Touchintensitaet_3": 10,
      "KLR_Touchauswertung": 10,
    })
  return packer.make_can_msg("KLR_01", bus, values)
