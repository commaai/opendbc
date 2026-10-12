from opendbc.car.volkswagen.mqbcan import (volkswagen_mqb_meb_checksum, xor_checksum,
                                           create_lka_hud_control as mqb_create_lka_hud_control)

# TODO: Parameterize the hca control type (5 vs 7) and consolidate with MQB (and PQ?)
def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "HCA_01_Status_HCA": 7 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control):
  return mqb_create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "LS_Hauptschalter",
    "LS_Typ_Hauptschalter",
    "LS_Codierung",
    "LS_Tip_Stufe_2",
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("LS_01", bus, values)


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


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  commands = []

  acc_01_values = {
    "ACC_Status_ACC": acc_control,
    "ACC_Sollbeschleunigung": accel if acc_enabled else 0,
    "ACC_zul_Regelabw_unten": 0.2,
    "ACC_zul_Regelabw_oben": 0.2,
    "ACC_neg_Sollbeschl_Grad": 4.0 if acc_enabled else 0,
    "ACC_pos_Sollbeschl_Grad": 4.0 if acc_enabled else 0,
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
    "ACC_Dynamik": 2,
    "ACC_Minimale_Bremsung": stopping,
  }
  commands.append(packer.make_can_msg("ACC_01", bus, acc_01_values))

  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, hud_control, mlb_hud_text):

  acc_active = acc_hud_status in (3, 4)
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.04,
    "ACC_Display_Prio": 0,
    "ACC_Anzeige_Zeitluecke": 1 if acc_active else 0,
    "ACC_Gesetzte_Zeitluecke": hud_control.leadDistanceBars, # TODO: Update openpilot charisma using stock rocker switch
    "ACC_Tachokranz": 1 if acc_active else 0,
    "ACC_Relevantes_Objekt": 2 if hud_control.visualAlert > 0 else (1 if acc_active and hud_control.leadVisible else 0),
    "ACC_Status_Prim_Anz": 2 if hud_control.visualAlert > 0 else (1 if acc_active else 0),
    "ACC_Akustik": 1 if hud_control.audibleAlert == 5 else 0, # Audible alert on OP warningImmediate
    "ACC_Abstandsindex": 1023 if acc_active else 1022,
    "ACC_Texte_Primaeranz": mlb_hud_text,
  }

  return packer.make_can_msg("ACC_02", bus, values)


def volkswagen_mlb_checksum(address: int, sig, d: bytearray) -> int:
  xor_starting_value = {
    0x109: 0x08, # ACC_01
    0x111: 0x10, # TSK_05
    0x30C: 0x0F, # ACC_02
    0x324: 0x27, # ACC_04
    0x10B: 0xA,  # LS_01
    0x10D: 0x0C, # ACC_05
    0x10F: 0x0E, # ACC_0x10F
    0x311: 0x12, # ACC_0x311
    0x397: 0x94, # LDW_02
    0x10C: 0x0D, # TSK_02
  }
  if address in xor_starting_value:
    return xor_checksum(address, sig, d, xor_starting_value[address])
  else:
    return volkswagen_mqb_meb_checksum(address, sig, d)
