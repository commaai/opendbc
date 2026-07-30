from opendbc.car import Bus, structs
from opendbc.can import CANDefine
from opendbc.car.volkswagen.values import DBC

LongCtrlState = structs.CarControl.Actuators.LongControlState


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


ACC_HUD_ERROR    = 6
ACC_HUD_OVERRIDE = 4
ACC_HUD_ACTIVE   = 3
ACC_HUD_ENABLED  = 2
ACC_HUD_DISABLED = 0


class MebLongStateMachine:
  def __init__(self, CP, CCP):
    self.CCP = CCP
    self.RAMP_FRAMES = 10 // CCP.ACC_CONTROL_STEP  # 100 ms

    self.ramp_counter = 0

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.acc_status_vals = {v: k for k, v in can_define.dv['ACC_18']['ACC_Status_ACC'].items()}
    self.acc_hold_type_vals = {v: k for k, v in can_define.dv['ACC_18']['ACC_Anforderung_HMS'].items()}

    self.prev_acc_hold_type = self.acc_hold_type_vals['KEINE_ANFORDERUNG']  # no request
    self.acc_status = self.acc_status_vals['ACC_OFF_HAUPTSCHALTER_AUS']  # last acc status, read by HUD msg

  def _get_acc_status(self, CS, CC) -> int:
    # stateless
    if CS.out.accFaulted:
      return self.acc_status_vals['REVERSIBLER_FEHLER_IM_ACC_SYSTEM']
    elif CC.enabled:
      return self.acc_status_vals['ACC_OVERRIDE' if CC.cruiseControl.override else 'ACC_AKTIV_REGELT']
    elif CS.out.cruiseState.available:
      return self.acc_status_vals['ACC_STANDBY']
    else:
      return self.acc_status_vals['ACC_OFF_HAUPTSCHALTER_AUS']  # disabled

  def _get_hold_type(self, CS, CC) -> int:
    # warning: car is reacting to hold mechanic even with long control off
    # NOTE: this allows KEINE_ANFORDERUNG -> ANFAHREN, but we haven't observed a fault due to this yet
    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    starting = CC.actuators.longControlState == LongCtrlState.pid and CS.esp_hold_confirmation

    if CS.out.accFaulted or not CC.longActive:
      acc_hold_type = self.acc_hold_type_vals['KEINE_ANFORDERUNG']  # no request
    elif stopping:
      acc_hold_type = self.acc_hold_type_vals['HALTEN']  # stopping/stopped
    elif starting:
      acc_hold_type = self.acc_hold_type_vals['ANFAHREN']  # resume after reaching full stop
    else:
      acc_hold_type = self.acc_hold_type_vals['KEINE_ANFORDERUNG']  # no request

    # enforce legal transitions
    if acc_hold_type == self.acc_hold_type_vals['HALTEN']:
      # allow going into hold at any time, reset ramp counter
      self.ramp_counter = 0
    elif self.prev_acc_hold_type == self.acc_hold_type_vals['HALTEN'] and acc_hold_type == self.acc_hold_type_vals['KEINE_ANFORDERUNG']:
      # HALTEN -> NONE causes car to fault into park. this enforces HALTEN -> RAMP if user overrides, or
      # if we requested to hold but never hit standstill before wanting to go again, we match stock and send just RAMP.
      acc_hold_type = self.acc_hold_type_vals['LOESEN_UEBER_RAMPE']
      self.ramp_counter = self.RAMP_FRAMES
    elif self.ramp_counter > 0:
      acc_hold_type = self.acc_hold_type_vals['LOESEN_UEBER_RAMPE']
      self.ramp_counter -= 1

    return acc_hold_type

  def update(self, CS, CC, accel) -> tuple[float, int, int, bool]:
    acc_status = self._get_acc_status(CS, CC)
    acc_hold_type = self._get_hold_type(CS, CC)

    # transition to inactive accel and jerks as soon as we enter ESP standstill
    requesting_hold = acc_hold_type == self.acc_hold_type_vals['HALTEN']
    held = requesting_hold and CS.esp_hold_confirmation
    if not CC.enabled or held:
      accel = self.CCP.ACCEL_INACTIVE

    # hold requested but the car hasn't reached standstill yet
    braking_to_stop = requesting_hold and not CS.esp_hold_confirmation

    self.prev_acc_hold_type = acc_hold_type
    self.acc_status = acc_status
    return accel, acc_status, acc_hold_type, braking_to_stop


class _ReproCarControl:
  """REPRO ONLY. Stand-in for the immutable capnp CarControl reader with longControlState replaced."""
  def __init__(self, CC, long_control_state):
    self.enabled = CC.enabled
    self.longActive = CC.longActive
    self.cruiseControl = CC.cruiseControl
    self.actuators = structs.CarControl.Actuators(longControlState=long_control_state)


class MebCreepChurnRepro:
  """REPRO ONLY, do not merge. Reproduces the TSK permanent fault from 000000b6--48c9d2f02a/23
  (t=35.26) and 000000b8--ab902978ef/9 (t=50.15).

  In both routes the policy chattered longControlState pid/stopping at ~6.5 Hz while the car was
  CREEPING, not while parked. b6 crept at 0.06-0.12 m/s for 1.3 s with the hold off, then churned
  another 2.2 s with it on, then TSK went to 7. So the stimulus is the car repeatedly grabbing and
  releasing the ESP hold, which a pulse train against a car that never moves cannot produce.

  So drive it closed loop: a P controller onto a creep target, plus the pid/stopping chatter fed
  into the REAL MebLongStateMachine. Everything downstream (which HMS value, the RAMP insertion,
  the accel zeroing, braking_to_stop) is production code, so this cannot emit a transition master
  could not, and the two phases of the real fault fall out on their own: with the hold off "go"
  legalizes to RAMP then KEINE_ANFORDERUNG, with it on "go" is ANFAHREN.

  Two rates. The fast one is the churn itself, at b6's measured rate. The slow one alternates
  creeping with sitting stopped, because that is the shape of the real fault and neither half
  reproduces on its own: the creep is what makes the ESP hold grab and let go, and the stop is
  where both faults actually fired.

  Brake or gas hands the car straight back to the policy, so the driver always wins.
  """
  ARM_SPEED = 10.0     # m/s, engage anywhere under this and the harness brakes the car down itself
  CHURN_SPEED = 2.0    # m/s, above this hold a steady stop request so the approach is a normal stop
  CREEP_SPEED = 0.15   # m/s, creep target. b6 crept at 0.06-0.12 through its first phase
  KP = 4.0             # lands the creep on 0.07-0.15 m/s, and doubles as the approach brake
  ACCEL_MIN = -1.5
  ACCEL_MAX = 0.6
  POKE_ACCEL = 0.12    # what b6 sent with ANFAHREN in its second phase, too weak to break the hold
  STOP_ACCEL = -0.55   # MEB stopAccel, interface.py. the creep needs a real stop request to end
  GO_FRAMES = 10       # 200 ms, b6 averaged 0.21 s in ANFAHREN
  STOP_FRAMES = 5      # 100 ms, b6 averaged 0.10 s in HALTEN, so 6.7 transitions/s against b6's 6.5
  CREEP_FRAMES = 75    # 1.5 s creeping, b6's first phase ran 1.26 s
  HOLD_FRAMES = 100    # then 2 s stopped, b6's second ran 2.16 s

  def __init__(self, CP, CCP):
    self.frames = 0

  def update(self, CS, CC, accel):
    if not CC.enabled or CS.out.brakePressed or CS.out.gasPressed or CS.out.vEgo > self.ARM_SPEED:
      self.frames = 0
      return accel, CC

    self.frames += 1
    creeping = self.frames % (self.CREEP_FRAMES + self.HOLD_FRAMES) < self.CREEP_FRAMES
    going = CS.out.vEgo < self.CHURN_SPEED and self.frames % (self.GO_FRAMES + self.STOP_FRAMES) < self.GO_FRAMES

    accel = min(max((self.CREEP_SPEED - CS.out.vEgo) * self.KP, self.ACCEL_MIN), self.ACCEL_MAX)
    if not creeping:
      accel = self.POKE_ACCEL if going else min(accel, self.STOP_ACCEL)
    elif not going:
      accel = min(accel, 0.0)  # coast, braking every 200 ms would take the creep away
    return accel, _ReproCarControl(CC, LongCtrlState.pid if going else LongCtrlState.stopping)


def create_acc_accel_control(packer, bus, CCP, acc_type, acc_enabled, accel, acc_status, acc_hold_type,
                             braking_to_stop, speed, travel_assist_available):
  # active longitudinal control disables one pedal driving (regen mode) while using overriding mechanism
  # error mitigation when stopping or stopped: (newer gen cars can be very sensitive)
  # - send 0 m stopping distance for cars in kind of parameterized stopping mode (stopping accel -0.2 seen for those cars)
  # -> this mode is seen for different cars with same firmware radars so could be a coded operational mode
  # - jerk and control limits values set inactive together when fully stopped
  # - set accel to 0 / no stop accel for full stop (seems to be compatible with old (non 0 stop accel) and new gen, because HMS state holds the car anyways)
  # - stopping command sent while requesting stop but ESP is not in standstill
  commands = []

  # ACC_Anhalteweg: when stopping: MEB: values <> 0 the car can execute a hard brake probably if target is too close, MQBEvo: value 0 results in hard brake
  terminal_rollout = 0

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_status,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  accel,
    "ACC_zul_Regelabw_unten":     0,
    "ACC_zul_Regelabw_oben":      0,
    "ACC_neg_Sollbeschl_Grad_02": CCP.JERK_LIMIT if accel != CCP.ACCEL_INACTIVE else 0,
    "ACC_pos_Sollbeschl_Grad_02": CCP.JERK_LIMIT if accel != CCP.ACCEL_INACTIVE else 0,
    "ACC_Anfahren":               0,  # always zero, stock uses ACC_Anforderung_HMS
    "ACC_Anhalten":               1 if braking_to_stop else 0,
    "ACC_Anhalteweg":             terminal_rollout if braking_to_stop else 20.46,
    "ACC_Anforderung_HMS":        acc_hold_type,
    "ACC_AKTIV_regelt":           0,  # always zero, stock uses ACC_Status_ACC
    "Speed":                      speed,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }

  commands.append(packer.make_can_msg("ACC_18", bus, values))

  if travel_assist_available:
    # satisfy car to prevent errors when pressing Travel Assist Button
    values_ta = {
       "Travel_Assist_Status":    4 if acc_enabled else 2,
       "Travel_Assist_Request":   0,
       "Travel_Assist_Available": 1,
    }

    commands.append(packer.make_can_msg("TA_01", bus, values_ta))

  return commands


def create_acc_hud_control(packer, bus, acc_status, set_speed, lead_visible, distance_bars, show_distance_bars, distance, fcw_alert):
  values = {
    "ACC_Status_ACC":                acc_status,
    "ACC_Tempolimit":                0,
    "ACC_Wunschgeschw_02":           set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke":       distance_bars, # 5 distance bars available (3 are used by OP)
    "ACC_Display_Prio":              0 if fcw_alert and acc_status in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 1, # probably keeping warning in front
    "ACC_Optischer_Fahrerhinweis":   1 if fcw_alert and acc_status in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0, # enables optical warning
    "ACC_Akustischer_Fahrerhinweis": 3 if fcw_alert and acc_status in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0, # enables sound warning
    "ACC_Texte_Zusatzanz_02":        11 if fcw_alert and acc_status in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0, # type of warning: Break!
    "ACC_Abstandsindex_02":          569, # seems to be default for MEB but is not static in every case
    "ACC_EGO_Fahrzeug":              2 if fcw_alert and acc_status in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else
                                     (1 if acc_status == ACC_HUD_ACTIVE else 0), # red car warn symbol for fcw
    "Lead_Type_Detected":            1 if lead_visible else 0, # object should be displayed
    "Lead_Type":                     3 if lead_visible else 0, # displaying a car
    "Lead_Distance":                 distance if lead_visible else 0, # hud distance of object
    "ACC_Enabled":                   1 if acc_status in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0,
    "ACC_Standby_Override":          1 if acc_status != ACC_HUD_ACTIVE else 0,
    "Street_Color":                  1 if acc_status in (ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0, # light grey (1) or dark (0) street
    "Lead_Brightness":               3 if acc_status == ACC_HUD_ACTIVE else 0, # object shows in color
    # TODO: a nice speed dependent bar distance
    "Zeitluecke_1":                  0, # desired distance to lead object for distance bar 1
    "Zeitluecke_2":                  0, # desired distance to lead object for distance bar 2
    "Zeitluecke_3":                  0, # desired distance to lead object for distance bar 3
    "Zeitluecke_4":                  0, # desired distance to lead object for distance bar 4
    "Zeitluecke_5":                  0, # desired distance to lead object for distance bar 5
    "Zeitluecke_Farbe":              1 if acc_status in (ACC_HUD_ENABLED, ACC_HUD_ACTIVE, ACC_HUD_OVERRIDE) else 0, # yellow (1) or white (0) time gap
    "ACC_Anzeige_Zeitluecke":        show_distance_bars if acc_status != ACC_HUD_DISABLED else 0, # show distance bar selection
    "SET_ME_0X1":                    0x1,    # unknown
    "SET_ME_0X6A":                   0x6A,   # unknown
    "SET_ME_0XFFFF":                 0xFFFF, # unknown
    "SET_ME_0X7FFF":                 0x7FFF, # unknown
  }

  return packer.make_can_msg("ACC_19", bus, values)


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
