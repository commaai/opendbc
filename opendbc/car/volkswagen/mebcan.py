from opendbc.car.common.conversions import Conversions as CV

ACC_CTRL_ERROR    = 6
ACC_CTRL_OVERRIDE = 4
ACC_CTRL_ACTIVE   = 3
ACC_CTRL_ENABLED  = 2
ACC_CTRL_DISABLED = 0

ACC_HMS_RAMP_RELEASE = 5
ACC_HMS_RELEASE      = 4
ACC_HMS_HOLD         = 1
ACC_HMS_NO_REQUEST   = 0

ACC_HUD_ERROR    = 6
ACC_HUD_OVERRIDE = 4
ACC_HUD_ACTIVE   = 3
ACC_HUD_ENABLED  = 2
ACC_HUD_DISABLED = 0


def create_steering_control(packer, bus, apply_curvature, lkas_enabled, power, power_boost):
  # active lateral control deactivates active steering wheel centering 
  values = {
    "Curvature": abs(apply_curvature) * CV.RAD_TO_DEG, # in deg/m
    "VZ": 1 if apply_curvature > 0 and lkas_enabled else 0,
    "Power": power if lkas_enabled else 0,
    "Power_Boost": 1 if power_boost and lkas_enabled else 0,
    "Active": lkas_enabled,
    "Request": lkas_enabled,
    "Standby": not lkas_enabled,
  }
  return packer.make_can_msg("HCA_03", bus, values)


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
  

def acc_control_value(main_switch_on, acc_faulted, long_active, esp_hold, override):

  if acc_faulted:
    acc_control = ACC_CTRL_ERROR # error state
  elif long_active:
    if override:
      acc_control = ACC_CTRL_OVERRIDE # overriding
    else:
      acc_control = ACC_CTRL_ACTIVE # active long control state
  elif main_switch_on:
    acc_control = ACC_CTRL_ENABLED # long control ready
  else:
    acc_control = ACC_CTRL_DISABLED # long control deactivated state

  return acc_control


def acc_hold_type(main_switch_on, acc_faulted, long_active, acc_hold_type_prev, starting, stopping, esp_hold, override):
  # warning: car is reacting to hold mechanic even with long control off

  if acc_faulted or not long_active:
    acc_hold_type = ACC_HMS_NO_REQUEST # no hold request
  elif override:
    if acc_hold_type_prev != ACC_HMS_NO_REQUEST:
      acc_hold_type = ACC_HMS_RAMP_RELEASE # ramp release of requests at the beginning of override
    else:
      acc_hold_type = ACC_HMS_NO_REQUEST # overriding / no request
  elif starting:
    acc_hold_type = ACC_HMS_RELEASE # release request and startup
  elif stopping or esp_hold:
    acc_hold_type = ACC_HMS_HOLD # hold or hold request
  else:
    acc_hold_type = ACC_HMS_NO_REQUEST # no hold request

  return acc_hold_type


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, acc_hold_type, stopping, starting, esp_hold, override, travel_assist_available):
  # active longitudinal control disables one pedal driving (regen mode of accelerator) while using overriding mechnism
  commands = []

  if acc_enabled:
    if override: # the car expects a non inactive accel while overriding
      acceleration = 0.00
    else:
      acceleration = accel
  else:
    acceleration = 3.01 # inactive accel

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_control,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  acceleration,
    "ACC_zul_Regelabw_unten":     0.2 if acc_control == ACC_CTRL_ACTIVE else 0,
    "ACC_zul_Regelabw_oben":      0.2 if acc_control == ACC_CTRL_ACTIVE else 0,
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_control == ACC_CTRL_ACTIVE else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if acc_control == ACC_CTRL_ACTIVE else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_Anfahren":               starting,
    "ACC_Anhalten":               stopping,
    "ACC_Anhalteweg":             20.46,
    "ACC_Anforderung_HMS":        acc_hold_type,
    "ACC_AKTIV_regelt":           1 if acc_control == ACC_CTRL_ACTIVE else 0,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }

  commands.append(packer.make_can_msg("MEB_ACC_02", bus, values))

  if travel_assist_available:
    # satisfy car to prevent errors when pressing Travel Assist Button
    values_ta = {
       "Travel_Assist_Status":    4 if acc_enabled else 2,
       "Travel_Assist_Request":   0,
       "Travel_Assist_Available": 1,
    }

    commands.append(packer.make_can_msg("MEB_Travel_Assist_01", bus, values_ta))

  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, esp_hold, override):

  if acc_faulted:
    acc_hud_control = ACC_HUD_ERROR # error state
  elif long_active:
    if override:
      acc_hud_control = ACC_HUD_OVERRIDE # overriding
    else:
      acc_hud_control = ACC_HUD_ACTIVE # active
  elif main_switch_on:
    acc_hud_control = ACC_HUD_ENABLED # inactive
  else:
    acc_hud_control = ACC_HUD_DISABLED # deactivated

  return acc_hud_control


def get_desired_gap(distance_bars, desired_gap, current_gap_signal):
  # mapping desired gap to correct signal of corresponding distance bar
  gap = 0
  
  if distance_bars == current_gap_signal:
    gap = desired_gap 

  return gap


def create_acc_hud_control(packer, bus, acc_control, set_speed, lead_visible, distance_bars, desired_gap, distance, esp_hold):

  values = {
    "ACC_Status_ACC":          acc_control,
    "ACC_Wunschgeschw_02":     set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": distance_bars, # 5 distance bars available (3 are used by OP)
    "ACC_Display_Prio":        1,
    "ACC_Abstandsindex_02":    569,
    "ACC_EGO_Fahrzeug":        1 if acc_control == ACC_HUD_ACTIVE else 0,
    "Lead_Type_Detected":      1 if lead_visible else 0, # object should be displayed
    "Lead_Type":               3 if lead_visible else 0, # displaying a car
    "Lead_Distance":           distance if lead_visible else 0, # hud distance of object
    "ACC_Enabled":             1 if acc_control == ACC_HUD_ACTIVE else 0,
    "ACC_Standby_Override":    1 if acc_control != ACC_HUD_ACTIVE else 0,
    "ACC_AKTIV_regelt":        1 if acc_control == ACC_HUD_ACTIVE else 0,
    "Lead_Brightness":         3 if acc_control == ACC_HUD_ACTIVE else 0, # object shows in colour
    "ACC_Events":              3 if esp_hold and acc_control == ACC_HUD_ACTIVE else 0, # acc ready message at standstill
    "Zeitluecke_1":            get_desired_gap(distance_bars, desired_gap, 1), # desired distance to lead object for distance bar 1
    "Zeitluecke_2":            get_desired_gap(distance_bars, desired_gap, 2), # desired distance to lead object for distance bar 2
    "Zeitluecke_3":            get_desired_gap(distance_bars, desired_gap, 3), # desired distance to lead object for distance bar 3
    "Zeitluecke_4":            get_desired_gap(distance_bars, desired_gap, 4), # desired distance to lead object for distance bar 4
    "Zeitluecke_5":            get_desired_gap(distance_bars, desired_gap, 5), # desired distance to lead object for distance bar 5
    "Zeitluecke_Farbe":        1 if acc_control in (ACC_HUD_ENABLED, ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0, # yellow (1) or white (0) time gap
    "SET_ME_0X1":              0x1,    # unknown
    "SET_ME_0X6A":             0x6A,   # unknown
    "SET_ME_0X3FF":            0x3FF,  # unknown
    "SET_ME_0XFFFF":           0xFFFF, # unknown
    "SET_ME_0X7FFF":           0x7FFF, # unknown
  }

  return packer.make_can_msg("MEB_ACC_01", bus, values)
