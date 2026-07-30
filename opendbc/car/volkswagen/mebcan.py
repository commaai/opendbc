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


class MebHoldPulseRepro:
  """REPRO ONLY, do not merge. Reproduces the TSK permanent fault seen at a standstill in
  000000b6--48c9d2f02a/23 (t=35.26) and 000000b8--ab902978ef/9 (t=50.15).

  In both routes the policy chattered pid/stopping while fully stopped, so we alternated
  ANFAHREN + 0.12 m/s^2 with HALTEN + ACCEL_INACTIVE about 5x/s. Measured dwells: ANFAHREN
  0.10-0.34 s, HALTEN 0.04-0.16 s, 8 cycles over 2.2 s, then TSK -> 7 while sitting in HALTEN.

  Ladder, cheapest theory first:
    1. CYCLES=1, GAP=5  one 100 ms ANFAHREN, the shortest ANFAHREN block in the logs. TESTED, does
                        not fault. Brakes audibly release, so the car does act on it, meaning a
                        single short drive-off request is not by itself illegal.
    2. CYCLES=2, GAP=2  adds one 40 ms HALTEN, the shortest block of any kind in the logs and one
                        step 1 never produced. TESTED, does not fault. So no single transition or
                        dwell is illegal on its own.
    3. CYCLES=12        sustained 5 Hz churn, 2.4 s and 24 transitions, a little past b6 (8 cycles
                        over 2.2 s) and near b8 (25 transitions). If only this faults it is the
                        rate, not any single transition.  <-- current
    4. TOGGLE_ANHALTEN   step 3 clean too, so the HMS/accel pattern alone is not the trigger.
                        Measured over the 6 s before each fault: b6 toggled ACC_Anhalten and
                        ACC_Anhalteweg 8 times, b8 20 times, this harness 0. braking_to_stop is
                        requesting_hold and not esp_hold, and we only ever run at hld=1, so those
                        fields sat constant while the real faults cycled them 1<->0 and
                        0<->20.46 alongside the HMS. Same pulse train as step 3, plus that.  <-- current

  Waits for the driver to be off the brake so the car is held by ESP alone, matching both routes.
  longActive is already true while pre-enabled with the brake held, so without this the whole train
  elapses against the brake pedal and the car just drives off normally when you release it.

  Holds steady for SETTLE_FRAMES so the pulse train is the only stimulus, then hands control back to
  MebLongStateMachine so the car can drive off.
  """
  SETTLE_FRAMES = 50   # 1 s of steady HALTEN before pulsing
  PULSE_FRAMES = 10    # ANFAHREN 200 ms, b6 phase B averaged 0.21 s
  GAP_FRAMES = 5       # HALTEN 100 ms, b6 phase B averaged 0.10 s
  CYCLES = 7           # 2.1 s at 6.7 transitions/s, b6 phase B was 2.16 s at 6.5
  PULSE_ACCEL = 0.12   # accel sent with ANFAHREN, matches both routes
  TOGGLE_ANHALTEN = True  # cycle ACC_Anhalten 1<->0 and ACC_Anhalteweg 0<->20.46 with the train
  # 5 s of steady HALTEN after the train. both faults fired after the churn stopped, b6 0.38 s into
  # the following HALTEN block rather than during the churn itself
  HOLD_AFTER_FRAMES = 150
  # phase A, run on the approach while the hold is still off. block lengths from b6 30.88-32.14
  # both faults were engaged while ALREADY CREEPING, b6 at 0.11 m/s and b8 at 0.46 m/s, and b6's
  # churn began 0.14 s after the engage. engaging at a full stop skips this phase entirely because
  # esp_hold is already set, which is what every test so far did.
  APPROACH_SPEED = 1.0  # m/s, start churning once we are creeping in
  A_FRAMES = 100        # 2 s ceiling, b6 churned for 1.26 s. bounded: the churn can stop the hold latching
  A_HALTEN = 10         # 200 ms
  A_RAMP = 6            # 120 ms
  A_NONE = 1            # 20 ms, gives 8.8 transitions/s against b6's 8.7

  def __init__(self, CP, CCP):
    self.CCP = CCP
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    acc_hold_type_vals = {v: k for k, v in can_define.dv['ACC_18']['ACC_Anforderung_HMS'].items()}
    self.halten = acc_hold_type_vals['HALTEN']
    self.anfahren = acc_hold_type_vals['ANFAHREN']
    self.ramp = acc_hold_type_vals['LOESEN_UEBER_RAMPE']
    self.none = acc_hold_type_vals['KEINE_ANFORDERUNG']
    self.frames = 0
    self.approach_frames = 0

  def update(self, CS, CC, accel, acc_hold_type, braking_to_stop) -> tuple[float, int, bool]:
    # CC.enabled, not longActive: a gas press to induce the creep drops longActive and would hand
    # the policy straight back, which is the opposite of what we want here
    armed = CC.enabled and not CS.out.brakePressed and \
            (CS.esp_hold_confirmation or CS.out.vEgo < self.APPROACH_SPEED)
    if not armed:
      self.frames = self.approach_frames = 0
      return accel, acc_hold_type, braking_to_stop

    self.frames += 1

    # phase A, churn while the hold is still off. this is where both faults spent ~1.3 s and the
    # only phase that can make the car actually grab and release: a 100 ms ANFAHREN at hld=1 never
    # drops Standstill, so nothing hydraulic cycles. block lengths from b6 30.88-32.14.
    # bounded, since the churn itself can stop the hold from ever latching
    if self.frames <= self.A_FRAMES:
      if not CS.esp_hold_confirmation:
        i = self.frames % (self.A_HALTEN + self.A_RAMP + self.A_NONE)
        if i < self.A_HALTEN:
          return accel, self.halten, True
        if i < self.A_HALTEN + self.A_RAMP:
          return accel, self.ramp, False
        return accel, self.none, False
      return self.CCP.ACCEL_INACTIVE, self.halten, False  # already held, wait out the phase

    frame = self.frames - self.A_FRAMES - self.SETTLE_FRAMES
    period = self.PULSE_FRAMES + self.GAP_FRAMES

    if frame <= 0:
      return self.CCP.ACCEL_INACTIVE, self.halten, False  # settling, steady hold
    if frame > self.CYCLES * period + self.HOLD_AFTER_FRAMES:
      return accel, acc_hold_type, braking_to_stop  # done, hand back so the car can resume
    if frame > self.CYCLES * period:
      return self.CCP.ACCEL_INACTIVE, self.halten, False  # steady hold, the policy is not sent at all
    if (frame - 1) % period < self.PULSE_FRAMES:
      return self.PULSE_ACCEL, self.anfahren, False
    # the real faults cycled the stop request with the hold request, we cannot get there through
    # braking_to_stop because that needs esp_hold low, so force it alongside the HALTEN blocks
    return self.CCP.ACCEL_INACTIVE, self.halten, self.TOGGLE_ANHALTEN


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
