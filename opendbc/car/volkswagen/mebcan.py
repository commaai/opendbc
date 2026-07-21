from opendbc.car import structs

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


ACC_CTRL_ERROR    = 6
ACC_CTRL_OVERRIDE = 4
ACC_CTRL_ACTIVE   = 3
ACC_CTRL_ENABLED  = 2
ACC_CTRL_DISABLED = 0

ACC_HMS_RAMP_RELEASE = 5
ACC_HMS_RELEASE      = 4
ACC_HMS_HOLD         = 1
ACC_HMS_NO_REQUEST   = 0


class MebLongState:
  """MEB longitudinal command state (ACC_18).

  ACC_Status_ACC, ACC_Anforderung_HMS and ACC_Sollbeschleunigung_02 have to agree with each other every
  frame. The car faults, and can clamp the EPB into Park, on an illegal hold transition or on the inactive
  accel sentinel without a hold request, so all three are derived here from one state instead of being
  reconstructed independently per frame.

  A hold is only ever left through a release request, never dropped straight to NONE:
    NONE -> HALTEN -> {ANFAHREN (openpilot wants to go) | RAMP_RELEASE (override or disengage)} -> NONE
  Stock never once sends HALTEN->NONE, and it faults us when we do. Since the next request is keyed off the
  last one we actually sent, no amount of chatter out of the planner can produce an illegal transition.
  """
  RAMP_FRAMES = 5  # RAMP_RELEASE length, ~100ms at 50Hz

  def __init__(self, CCP):
    self.CCP = CCP
    self.enabled = False
    self.control = ACC_CTRL_DISABLED
    self.hold = ACC_HMS_NO_REQUEST
    self.accel = CCP.ACCEL_INACTIVE
    self.held = False    # car is physically stopped under our hold request
    self.settling = False  # hold requested, still braking down to a stop
    self.ramp = 0        # frames left of RAMP_RELEASE

  def update(self, CS, CC, accel):
    faulted = CS.out.accFaulted
    override = CC.cruiseControl.override
    esp_hold = CS.esp_hold_confirmation
    going = CC.actuators.longControlState == LongCtrlState.pid
    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    prev = self.hold

    # **** hold request **** #
    # a handshake already in progress finishes before the policy gets a say, so the car only ever sees the
    # transitions it accepts. Below that, the policy picks which of the legal successors we take

    if faulted:
      self.ramp = 0
      self.hold = ACC_HMS_NO_REQUEST  # the car has already given up, nothing left to hold
    elif prev == ACC_HMS_RAMP_RELEASE:
      self.ramp -= 1
      if self.ramp > 0:
        self.hold = ACC_HMS_RAMP_RELEASE
      else:
        self.hold = ACC_HMS_NO_REQUEST
    elif not CC.longActive:  # driver override or disengage
      if prev in (ACC_HMS_HOLD, ACC_HMS_RELEASE):
        self.ramp = self.RAMP_FRAMES
        self.hold = ACC_HMS_RAMP_RELEASE  # ramp out of whatever we were requesting
      else:
        self.hold = ACC_HMS_NO_REQUEST
    elif prev == ACC_HMS_RELEASE:  # driving off
      if not esp_hold:
        self.ramp = self.RAMP_FRAMES
        self.hold = ACC_HMS_RAMP_RELEASE  # rolling, drive off is done
      elif going:
        self.hold = ACC_HMS_RELEASE  # still waiting on the car to move
      else:
        self.hold = ACC_HMS_HOLD  # go was withdrawn before we moved, settle back into the hold
    elif going:  # openpilot wants to go
      if prev == ACC_HMS_HOLD:
        self.hold = ACC_HMS_RELEASE  # drive off
      elif esp_hold:
        self.hold = ACC_HMS_HOLD  # stopped without a hold, enter one so we can drive off legally
      else:
        self.hold = ACC_HMS_NO_REQUEST
    else:
      if prev == ACC_HMS_HOLD or stopping or esp_hold:
        self.hold = ACC_HMS_HOLD
      else:
        self.hold = ACC_HMS_NO_REQUEST

    self.held = esp_hold and self.hold == ACC_HMS_HOLD
    self.settling = self.hold == ACC_HMS_HOLD and not esp_hold

    # **** control state **** #

    self.enabled = CC.enabled
    if faulted:
      self.control = ACC_CTRL_ERROR  # error state
    elif CC.enabled:
      self.control = ACC_CTRL_OVERRIDE if override else ACC_CTRL_ACTIVE
    elif CS.out.cruiseState.available:
      self.control = ACC_CTRL_ENABLED  # long control ready
    else:
      self.control = ACC_CTRL_DISABLED  # long control deactivated state

    # **** acceleration **** #

    # once the car is holding itself we hand it the neutral sentinel and let its standstill logic hold, while
    # still settling we send the live decel (stock modulates it, up to hard braking, to catch the car). Stock
    # only ever pairs the sentinel with HALTEN, and sending it under any other request faults the car
    if not CC.enabled:
      self.accel = self.CCP.ACCEL_INACTIVE
    elif override:
      self.accel = self.CCP.ACCEL_OVERRIDE  # the car expects a non-inactive accel while overriding
    elif self.held:
      self.accel = self.CCP.ACCEL_INACTIVE
    else:
      self.accel = accel


def create_acc_accel_control(packer, bus, acc_type, state, speed, travel_assist_available):
  # active longitudinal control disables one pedal driving (regen mode) while using overriding mechanism
  commands = []

  jerk_limit = state.CCP.JERK_LIMIT if state.control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) and not state.held else 0

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             state.control,
    "ACC_StartStopp_Info":        state.enabled,
    "ACC_Sollbeschleunigung_02":  state.accel,
    "ACC_zul_Regelabw_unten":     0,
    "ACC_zul_Regelabw_oben":      0,
    "ACC_neg_Sollbeschl_Grad_02": jerk_limit,
    "ACC_pos_Sollbeschl_Grad_02": jerk_limit,
    "ACC_Anfahren":               0,  # always zero, stock uses ACC_Anforderung_HMS
    "ACC_Anhalten":               1 if state.settling else 0,
    "ACC_Anhalteweg":             0 if state.settling else 20.46,
    "ACC_Anforderung_HMS":        state.hold,
    "ACC_AKTIV_regelt":           1 if state.control == ACC_CTRL_ACTIVE else 0,
    "Speed":                      speed,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }

  commands.append(packer.make_can_msg("ACC_18", bus, values))

  if travel_assist_available:
    # satisfy car to prevent errors when pressing Travel Assist Button
    values_ta = {
       "Travel_Assist_Status":    4 if state.enabled else 2,
       "Travel_Assist_Request":   0,
       "Travel_Assist_Available": 1,
    }

    commands.append(packer.make_can_msg("TA_01", bus, values_ta))

  return commands


def get_desired_gap(distance_bars, desired_gap, current_gap_signal):
  # mapping desired gap to correct signal of corresponding distance bar
  gap = 0

  if distance_bars == current_gap_signal:
    gap = desired_gap

  return gap


def create_acc_hud_control(packer, bus, acc_control, set_speed, lead_visible, distance_bars, show_distance_bars, esp_hold, distance, desired_gap, fcw_alert):
  values = {
    "ACC_Status_ACC":                acc_control,
    "ACC_Tempolimit":                0,
    "ACC_Wunschgeschw_02":           set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke":       distance_bars, # 5 distance bars available (3 are used by OP)
    "ACC_Display_Prio":              0 if fcw_alert and acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) else 1, # probably keeping warning in front
    "ACC_Optischer_Fahrerhinweis":   1 if fcw_alert and acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) else 0, # enables optical warning
    "ACC_Akustischer_Fahrerhinweis": 3 if fcw_alert and acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) else 0, # enables sound warning
    "ACC_Texte_Zusatzanz_02":        11 if fcw_alert and acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) else 0, # type of warning: Break!
    "ACC_Abstandsindex_02":          569, # seems to be default for MEB but is not static in every case
    "ACC_EGO_Fahrzeug":              2 if fcw_alert and acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) else
                                     (1 if acc_control == ACC_CTRL_ACTIVE else 0), # red car warn symbol for fcw
    "Lead_Type_Detected":            1 if lead_visible else 0, # object should be displayed
    "Lead_Type":                     3 if lead_visible else 0, # displaying a car
    "Lead_Distance":                 distance if lead_visible else 0, # hud distance of object
    "ACC_Enabled":                   1 if acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) else 0,
    "ACC_Standby_Override":          1 if acc_control != ACC_CTRL_ACTIVE else 0,
    "Street_Color":                  1 if acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) else 0, # light grey (1) or dark (0) street
    "Lead_Brightness":               3 if acc_control == ACC_CTRL_ACTIVE else 0, # object shows in color
    "Zeitluecke_1":                  get_desired_gap(distance_bars, desired_gap, 1), # desired distance to lead object for distance bar 1
    "Zeitluecke_2":                  get_desired_gap(distance_bars, desired_gap, 2), # desired distance to lead object for distance bar 2
    "Zeitluecke_3":                  get_desired_gap(distance_bars, desired_gap, 3), # desired distance to lead object for distance bar 3
    "Zeitluecke_4":                  get_desired_gap(distance_bars, desired_gap, 4), # desired distance to lead object for distance bar 4
    "Zeitluecke_5":                  get_desired_gap(distance_bars, desired_gap, 5), # desired distance to lead object for distance bar 5
    "Zeitluecke_Farbe":              1 if acc_control in (ACC_CTRL_ENABLED, ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE) else 0, # yellow (1) or white (0) time gap
    "ACC_Anzeige_Zeitluecke":        show_distance_bars if acc_control != ACC_CTRL_DISABLED else 0, # show distance bar selection
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
