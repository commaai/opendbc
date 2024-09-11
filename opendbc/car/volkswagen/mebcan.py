from opendbc.car.common.numpy_fast import clip

def create_steering_control_curvature(packer, bus, apply_curvature, lkas_enabled, power):
  values = {
    #"Curvature": abs(apply_curvature) * 1000, # in 1/mm
    "Steering_Angle": abs(apply_curvature),
    "VZ": 1 if apply_curvature < 0 and lkas_enabled == 1 else 0, # > for curvature
    "Power": power if lkas_enabled else 0,
    "Active": lkas_enabled,
    "Active_02": lkas_enabled,
    "Inactive": not lkas_enabled,
  }
  return packer.make_can_msg("HCA_03", bus, values)


def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_Request": lkas_enabled,
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 and lkas_enabled == 1 else 0,
    "HCA_01_Enable": lkas_enabled,
    "HCA_01_Standby": not lkas_enabled,
    "HCA_01_Available": 1,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control, sound_alert):
  display_mode = 1 if lat_active else 0 # travel assist style showing yellow lanes when op is active
  
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
  

def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
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
    "GRA_Tip_Wiederaufnahme": resume,
  })
  return packer.make_can_msg("GRA_ACC_01", bus, values)
  

def acc_control_value(main_switch_on, acc_faulted, long_active, just_disabled):
  if acc_faulted:
    acc_control = 6
  elif just_disabled:
    acc_control = 5
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control
  

def acc_hold_type(main_switch_on, acc_faulted, long_active, just_disabled, starting, stopping, esp_hold):
  if acc_faulted or not main_switch_on:
    acc_hold_type = 0
  elif just_disabled: # prevent errors
    acc_hold_type = 5 # disable acc confirmation
  elif starting:
    acc_hold_type = 4 # hold release and startup
  elif stopping:
    acc_hold_type = 1 # hold or hold request
  elif esp_hold and long_active:
    acc_hold_type = 1 # hold
  else:
    acc_hold_type = 0

  return acc_hold_type
  

def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, acc_hold_type, stopping, starting, lower_jerk, upper_jerk, esp_hold, speed, reversing, meb_acc_02_values):
  LONG_ACTIVE = 3
  commands = []

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_control,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  accel if acc_enabled else 3.01,
    "ACC_zul_Regelabw_unten":     max(0.05, lower_jerk) if acc_enabled else 0,
    "ACC_zul_Regelabw_oben":      min(3.0, upper_jerk) if acc_enabled else 0,
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_Anfahren":               starting,
    "ACC_Anhalten":               stopping,
    "ACC_Anhalteweg":             20.46,
    "ACC_Anforderung_HMS":        acc_hold_type,
    "ACC_AKTIV_regelt":           1 if acc_control == LONG_ACTIVE else 0,
    "Speed":                      speed, # dont know if neccessary
    "Reversing":                  reversing, # dont know if neccessary
  }

  values.update({
    "SET_ME_0XFE": meb_acc_02_values["SET_ME_0XFE"],
    "SET_ME_0X1": meb_acc_02_values["SET_ME_0X1"],
    "SET_ME_0X9": meb_acc_02_values["SET_ME_0X9"],
    "SET_ME_0XFE": meb_acc_02_values["SET_ME_0XFE"],
    "SET_ME_0X1": meb_acc_02_values["SET_ME_0X1"],
    "SET_ME_0X9": meb_acc_02_values["SET_ME_0X9"],
  })
  
  commands.append(packer.make_can_msg("MEB_ACC_02", bus, values))

  # satisfy car to prevent errors when pressing Travel Assist Button
  # the button does nothing with this
  values_ta = {
     "Travel_Assist_Status" : 2, # ready
     "Travel_Assist_Request" : 0, # no request
     "Travel_Assist_Available" : 1,
  }

  commands.append(packer.make_can_msg("MEB_Travel_Assist_01", bus, values_ta))
  
  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, override):
  if acc_faulted:
    acc_hud_control = 6
  elif long_active:
    acc_hud_control = 4 if override else 3
  elif main_switch_on:
    acc_hud_control = 2
  else:
    acc_hud_control = 0

  return acc_hud_control
  

def create_acc_hud_control(packer, bus, acc_control, set_speed, lead_visible, gap, heartbeat, esp_hold, meb_acc_01_values):  
  lead_detected_by_car = True if meb_acc_01_values["Lead_Type_Detected"] > 0 else False
  lead_detected_by_op_only = lead_visible and not lead_detected_by_car
  LONG_ACTIVE = 3
  
  values = {
    #"STA_Primaeranz": acc_hud_status,
    "ACC_Status_ACC":          acc_control,
    "ACC_Wunschgeschw_02":     set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": gap,
    "ACC_Display_Prio":        1,
    "ACC_Abstandsindex_02":    512,
    "ACC_EGO_Fahrzeug":        1 if acc_control == LONG_ACTIVE else 0,
    "Heartbeat":               heartbeat, # do the same as radar would do, still check if this is necessary
    "Lead_Type_Detected":      1 if lead_detected_by_op_only else meb_acc_01_values["Lead_Type_Detected"], # object should be displayed
    "Lead_Type":               3 if lead_detected_by_op_only else meb_acc_01_values["Lead_Type"], # displaying a car
    "Lead_Distance":           80 if lead_detected_by_op_only else meb_acc_01_values["Lead_Distance"], # hud distance of object
    "ACC_Enabled":             1 if acc_control == LONG_ACTIVE else 0,
    "ACC_Standby_Override":    1 if acc_control != LONG_ACTIVE else 0,
    "ACC_AKTIV_regelt":        1 if acc_control == LONG_ACTIVE else 0,
    "ACC_Limiter_Mode":        0,
    "Lead_Brightness":         3 if acc_control == LONG_ACTIVE else 0, # object shows in colour
    "Unknown_03":              106, # prevents errors
    "Unknown_01":              0, # prevents errors
    "Unknown_08":              0, # prevents errors
    "ACC_Special_Events":      3 if esp_hold and acc_control == LONG_ACTIVE else 0, # acc ready message at standstill
  }

  values.update({
    #"STA_Primaeranz": meb_acc_01_values["STA_Primaeranz"],
    "Unknown_Area_01": meb_acc_01_values["Unknown_Area_01"],
    "SET_ME_0X1": meb_acc_01_values["SET_ME_0X1"],
    "SET_ME_0X3FF": meb_acc_01_values["SET_ME_0X3FF"],
    "SET_ME_0XFFFF": meb_acc_01_values["SET_ME_0XFFFF"],
    "SET_ME_0X7FFF": meb_acc_01_values["SET_ME_0X7FFF"],
    "Unknown_04": meb_acc_01_values["Unknown_04"],
    "Unknown_05": meb_acc_01_values["Unknown_05"],
    "Unknown_06": meb_acc_01_values["Unknown_06"],
    "Unknown_07": meb_acc_01_values["Unknown_07"],
    "Zeitluecke_1_Signal": meb_acc_01_values["Zeitluecke_1_Signal"],
    "Zeitluecke_2_Signal": meb_acc_01_values["Zeitluecke_2_Signal"],
    "Zeitluecke_3_Signal": meb_acc_01_values["Zeitluecke_3_Signal"],
    "Zeitluecke_4_Signal": meb_acc_01_values["Zeitluecke_4_Signal"],
    "Zeitluecke_5_Signal": meb_acc_01_values["Zeitluecke_5_Signal"],
    "ACC_Anzeige_Zeitluecke": meb_acc_01_values["ACC_Anzeige_Zeitluecke"],
  })

  return packer.make_can_msg("MEB_ACC_01", bus, values)
