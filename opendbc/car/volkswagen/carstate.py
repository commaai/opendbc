from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import DBC, CanBus, NetworkLocation, TransmissionType, GearShifter, \
                                                      CarControllerParams, VolkswagenFlags

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.frame = 0
    self.eps_init_complete = False
    self.cruise_recovery_timer = 0
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}
    self.esp_hold_confirmation = False
    self.upscale_lead_car_signal = False
    self.eps_stock_values = False
    self.acc_type = 0
    self.force_rhd_for_bsm = False # TODO find RHD detection signals

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.CP.pcmCruise:
      for b in buttonEvents:
        # Enable OP long on falling edge of enable buttons
        if b.type in (ButtonType.setCruise, ButtonType.resumeCruise) and not b.pressed:
          return True
    return False

  def create_button_events(self, pt_cp, buttons):
    button_events = []

    for button in buttons:
      state = pt_cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events

  def update(self, can_parsers) -> structs.CarState:
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]
    ext_cp = pt_cp if self.CP.networkLocation == NetworkLocation.fwdCamera else cam_cp

    if self.CP.flags & VolkswagenFlags.PQ:
      return self.update_pq(pt_cp, cam_cp, ext_cp)
    elif self.CP.flags & VolkswagenFlags.MLB:
      return self.update_mlb(pt_cp, cam_cp, ext_cp)
    elif self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      return self.update_meb(pt_cp, cam_cp, ext_cp)

    ret = structs.CarState()

    if self.CP.transmissionType == TransmissionType.direct:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Motor_EV_01"]["MO_Waehlpos"], None))
    elif self.CP.transmissionType == TransmissionType.manual:
      if bool(pt_cp.vl["Gateway_72"]["BCM1_Rueckfahrlicht_Schalter"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Gateway_73"]["GE_Fahrstufe"], None))

    if True:
      # MQB-specific
      if self.CP.flags & VolkswagenFlags.KOMBI_PRESENT:
        self.upscale_lead_car_signal = bool(pt_cp.vl["Kombi_03"]["KBI_Variante"])  # Analog vs digital instrument cluster

      self.parse_wheel_speeds(ret,
        pt_cp.vl["ESP_19"]["ESP_VL_Radgeschw_02"],
        pt_cp.vl["ESP_19"]["ESP_VR_Radgeschw_02"],
        pt_cp.vl["ESP_19"]["ESP_HL_Radgeschw_02"],
        pt_cp.vl["ESP_19"]["ESP_HR_Radgeschw_02"],
      )

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        ret.carFaultedNonCritical = bool(cam_cp.vl["HCA_01"]["EA_Ruckfreigabe"]) or cam_cp.vl["HCA_01"]["EA_ACC_Sollstatus"] > 0  # EA

      ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
      brake_pedal_pressed = bool(pt_cp.vl["Motor_14"]["MO_Fahrer_bremst"])
      brake_pressure_detected = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
      ret.brakePressed = brake_pedal_pressed or brake_pressure_detected
      ret.parkingBrake = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])  # FIXME: need to include an EPB check as well

      ret.doorOpen = any([pt_cp.vl["Gateway_72"]["ZV_FT_offen"],
                          pt_cp.vl["Gateway_72"]["ZV_BT_offen"],
                          pt_cp.vl["Gateway_72"]["ZV_HFS_offen"],
                          pt_cp.vl["Gateway_72"]["ZV_HBFS_offen"],
                          pt_cp.vl["Gateway_72"]["ZV_HD_offen"]])

      if self.CP.enableBsm:
        # Infostufe: BSM LED on, Warnung: BSM LED flashing
        ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
        ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

      ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
      ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

      self.acc_type = ext_cp.vl["ACC_06"]["ACC_Typ"]
      self.esp_hold_confirmation = bool(pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"])
      acc_limiter_mode = ext_cp.vl["ACC_02"]["ACC_Gesetzte_Zeitluecke"] == 0
      speed_limiter_mode = bool(pt_cp.vl["TSK_06"]["TSK_Limiter_ausgewaehlt"])

      ret.cruiseState.available = pt_cp.vl["TSK_06"]["TSK_Status"] in (2, 3, 4, 5)
      ret.cruiseState.enabled = pt_cp.vl["TSK_06"]["TSK_Status"] in (3, 4, 5)
      ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw_02"] * CV.KPH_TO_MS if self.CP.pcmCruise else 0
      ret.accFaulted = pt_cp.vl["TSK_06"]["TSK_Status"] in (6, 7)

      ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Left"])
      ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Right"])

    # Shared logic
    ret.vEgoCluster = pt_cp.vl["Kombi_01"]["KBI_angez_Geschw"] * CV.KPH_TO_MS

    self.parse_mlb_mqb_steering_state(ret, pt_cp)

    ret.gasPressed = pt_cp.vl["Motor_20"]["MO_Fahrpedalrohwert_01"] > 0
    ret.espActive = bool(pt_cp.vl["ESP_21"]["ESP_Eingriff"])
    ret.espDisabled = pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"] != 0
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    ret.standstill = ret.vEgoRaw == 0
    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation
    ret.cruiseState.nonAdaptive = acc_limiter_mode or speed_limiter_mode
    if ret.cruiseState.speed > 90:
      ret.cruiseState.speed = 0

    self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}
    self.gra_stock_values = pt_cp.vl["GRA_ACC_01"]

    ret.buttonEvents = self.create_button_events(pt_cp, self.CCP.BUTTONS)

    ret.lowSpeedAlert = self.update_low_speed_alert(ret.vEgo)

    self.frame += 1
    return ret

  def update_pq(self, pt_cp, cam_cp, ext_cp) -> structs.CarState:
    ret = structs.CarState()

    # vEgo obtained from Bremse_1 vehicle speed rather than Bremse_3 wheel speeds because Bremse_3 isn't present on NSF
    ret.vEgoRaw = pt_cp.vl["Bremse_1"]["BR1_Rad_kmh"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]["LH3_BLW"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_BLWSign"])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]["LW1_Lenk_Gesch"] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]["LW1_Gesch_Sign"])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]["LH3_LM"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_LMSign"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)

    # Update gas, brakes, and gearshift.
    ret.gasPressed = pt_cp.vl["Motor_3"]["MO3_Pedalwert"] > 0
    ret.brake = pt_cp.vl["Bremse_5"]["BR5_Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]["MO2_BLS"])
    ret.parkingBrake = bool(pt_cp.vl["Kombi_1"]["Bremsinfo"])

    # Update gear and/or clutch position data.
    if self.CP.transmissionType == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_1"]["GE1_Wahl_Pos"], None))
    elif self.CP.transmissionType == TransmissionType.manual:
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Rueckfahr"])
      if reverse_light:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gate_Komf_1"]["GK1_Fa_Tuerkont"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_BT_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HL_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HR_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HD_Hauptraste"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_Status"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapters on Front Assist with Braking and City Emergency
    # Braking for the 2016 Passat NMS
    # TODO: deferred until we can collect data on pre-MY2016 behavior, AWV message may be shorter with fewer signals
    ret.stockFcw = False
    ret.stockAeb = False

    # Update ACC radar status.
    self.acc_type = ext_cp.vl["ACC_System"]["ACS_Typ_ACC"]
    ret.cruiseState.available = bool(pt_cp.vl["Motor_5"]["MO5_GRA_Hauptsch"])
    ret.cruiseState.enabled = pt_cp.vl["Motor_2"]["MO2_Sta_GRA"] in (1, 2)
    if self.CP.pcmCruise:
      ret.accFaulted = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_StaACC"] in (6, 7)
    else:
      ret.accFaulted = pt_cp.vl["Motor_2"]["MO2_Sta_GRA"] == 3

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    ret.cruiseState.speed = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_V_Wunsch"] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(300, pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_li"],
                                                                            pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_re"])
    ret.buttonEvents = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    self.gra_stock_values = pt_cp.vl["GRA_Neu"]

    # Additional safety checks performed in CarInterface.
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]["BR1_ESPASR_passive"])

    ret.lowSpeedAlert = self.update_low_speed_alert(ret.vEgo)

    self.frame += 1
    return ret

  def update_mlb(self, pt_cp, cam_cp, ext_cp) -> structs.CarState:
    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
      pt_cp.vl["ESP_03"]["ESP_VL_Radgeschw"],
      pt_cp.vl["ESP_03"]["ESP_VR_Radgeschw"],
      pt_cp.vl["ESP_03"]["ESP_HL_Radgeschw"],
      pt_cp.vl["ESP_03"]["ESP_HR_Radgeschw"],
    )

    ret.gasPressed = pt_cp.vl["Motor_03"]["MO_Fahrpedalrohwert_01"] > 0
    ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_03"]["GE_Waehlhebel"], None))

    # ACC okay but disabled (1), ACC ready (2), a radar visibility or other fault/disruption (6 or 7)
    # currently regulating speed (3), driver accel override (4), brake only (5)
    # TODO: get this from the drivetrain side instead, for openpilot long support later
    ret.cruiseState.available = ext_cp.vl["ACC_05"]["ACC_Status_ACC"] in (2, 3, 4, 5)
    ret.cruiseState.enabled = ext_cp.vl["ACC_05"]["ACC_Status_ACC"] in (3, 4, 5)
    ret.accFaulted = ext_cp.vl["ACC_05"]["ACC_Status_ACC"] in (6, 7)

    self.parse_mlb_mqb_steering_state(ret, pt_cp)

    ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0
    brake_pedal_pressed = bool(pt_cp.vl["Motor_03"]["MO_Fahrer_bremst"])
    brake_pressure_detected = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
    ret.brakePressed = brake_pedal_pressed or brake_pressure_detected
    ret.parkingBrake = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])
    ret.espDisabled = pt_cp.vl["ESP_01"]["ESP_Tastung_passiv"] != 0

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(300, pt_cp.vl["Gateway_11"]["BH_Blinker_li"],
                                                                            pt_cp.vl["Gateway_11"]["BH_Blinker_re"])

    ret.seatbeltUnlatched = pt_cp.vl["Gateway_06"]["AB_Gurtschloss_FA"] != 3
    ret.doorOpen = any([pt_cp.vl["Gateway_05"]["FT_Tuer_geoeffnet"],
                        pt_cp.vl["Gateway_05"]["BT_Tuer_geoeffnet"],
                        pt_cp.vl["Gateway_05"]["HL_Tuer_geoeffnet"],
                        pt_cp.vl["Gateway_05"]["HR_Tuer_geoeffnet"]])

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}
    self.gra_stock_values = pt_cp.vl["LS_01"]

    ret.buttonEvents = self.create_button_events(pt_cp, self.CCP.BUTTONS)

    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation
    ret.standstill = ret.vEgoRaw == 0

    self.frame += 1
    return ret
    
  def update_meb(self, pt_cp, cam_cp, ext_cp) -> structs.CarState:
    ret = structs.CarState()
    
    # Update vehicle speed and acceleration from ABS wheel speeds.
    self.parse_wheel_speeds(ret,
      pt_cp.vl["ESC_51"]["VL_Radgeschw"],
      pt_cp.vl["ESC_51"]["VR_Radgeschw"],
      pt_cp.vl["ESC_51"]["HL_Radgeschw"],
      pt_cp.vl["ESC_51"]["HR_Radgeschw"],
    )

    if self.CP.flags & VolkswagenFlags.KOMBI_PRESENT:
      ret.vEgoCluster = pt_cp.vl["Kombi_01"]["KBI_angez_Geschw"] * CV.KPH_TO_MS
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    # LWI_01, MEP_EPS_01 steering angle differs from real steering angle (dynamic steering)
    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg  = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque   = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed  = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    ret.steeringCurvature = -pt_cp.vl["QFK_01"]["Curvature"] * (1, -1)[int(pt_cp.vl["QFK_01"]["Curvature_VZ"])]
    
    ret.yawRate = -pt_cp.vl["ESC_50"]["Yaw_Rate"] * (1, -1)[int(pt_cp.vl["ESC_50"]["Yaw_Rate_Sign"])] * CV.DEG_TO_RAD
    
    # Update gear and/or clutch position data.
    if self.CP.flags & VolkswagenFlags.ALT_GEAR:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Gateway_73"]["GE_Fahrstufe"], None)) # (candidate for all plattforms MEB and MQB evo)
    else:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))
    drive_mode = ret.gearShifter == GearShifter.drive
    
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["QFK_01"]["LatCon_HCA_Status"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status, drive_mode=drive_mode)

    # VW Emergency Assist status tracking and mitigation
    self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
    self.klr_stock_values = pt_cp.vl["KLR_01"] if self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT else {}
    ret.carFaultedNonCritical = cam_cp.vl["EA_01"]["EA_Funktionsstatus"] in (3, 4, 5, 6) # emergency assist always present also if not coded

    # Update gas, brakes, and gearshift.
    #ret.gasPressed   = pt_cp.vl["Motor_54"]["Accelerator_Pressure"] > 0 # MQBevo offset is not reliable (fluctuation or different statically in small range)
    ret.gasPressed   = pt_cp.vl["Motor_51"]["Accel_Pedal_Pressure"] > 0 # detects accel pedal "a little bit" later than ["Motor_54"]["Accelerator_Pressure"]
    ret.brakePressed = bool(pt_cp.vl["Motor_14"]["MO_Fahrer_bremst"]) # includes regen braking
    ret.brake        = pt_cp.vl["ESC_51"]["Brake_Pressure"]

    ret.parkingBrake = pt_cp.vl["ESC_50"]["EPB_Status"] in (1, 4) # EPB closing or closed (candidate for all plattforms)
    #ret.parkingBrake = pt_cp.vl["Gateway_73"]["EPB_Status"] in (1, 4) # this signal is not working for newer models

    # Update door and trunk/hatch lid open status.
    doors = pt_cp.vl["ZV_02"] if bool(pt_cp.vl["Gateway_72"]["ZV_02_alt"]) else pt_cp.vl["Gateway_72"]
    ret.doorOpen = any([doors["ZV_FT_offen"],
                        doors["ZV_BT_offen"],
                        doors["ZV_HFS_offen"],
                        doors["ZV_HBFS_offen"],
                        doors["ZV_HD_offen"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3
    
    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      bsm_bus = pt_cp if self.CP.flags & (VolkswagenFlags.MEB_GEN2 | VolkswagenFlags.MQB_EVO) else ext_cp
      blindspot_driver    = bool(bsm_bus.vl["MEB_Side_Assist_01"]["Blind_Spot_Info_Driver"]) or bool(bsm_bus.vl["MEB_Side_Assist_01"]["Blind_Spot_Warn_Driver"])
      blindspot_passenger = bool(bsm_bus.vl["MEB_Side_Assist_01"]["Blind_Spot_Info_Passenger"]) or bool(bsm_bus.vl["MEB_Side_Assist_01"]["Blind_Spot_Warn_Passenger"])
      car_is_lhd = True if not self.force_rhd_for_bsm else False # TODO
      ret.leftBlindspot  = blindspot_driver if car_is_lhd else blindspot_passenger
      ret.rightBlindspot = blindspot_passenger if car_is_lhd else blindspot_driver

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_02"]

    ret.stockFcw = bool(ext_cp.vl["AWV_03"]["FCW_Active"]) # currently most plausible candidate
    ret.stockAeb = False #bool(pt_cp.vl["VMM_02"]["AEB_Active"]) TODO find correct signal

    self.acc_type                = ext_cp.vl["ACC_18"]["ACC_Typ"]
    self.travel_assist_available = bool(cam_cp.vl["TA_01"]["Travel_Assist_Available"])

    ret.cruiseState.available = pt_cp.vl["Motor_51"]["TSK_Status"] in (2, 3, 4, 5)
    ret.cruiseState.enabled   = pt_cp.vl["Motor_51"]["TSK_Status"] in (3, 4, 5)

    if self.CP.pcmCruise:
      # Cruise Control mode; check for distance UI setting from the radar.
      # ECM does not manage this, so do not need to check for openpilot longitudinal
      ret.cruiseState.nonAdaptive = bool(ext_cp.vl["ACC_19"]["ACC_Limiter_Mode"])
    else:
      # Speed limiter mode; ECM faults if we command ACC while not pcmCruise
      ret.cruiseState.nonAdaptive = bool(pt_cp.vl["Motor_51"]["TSK_Limiter_ausgewaehlt"])

    accFaulted = pt_cp.vl["Motor_51"]["TSK_Status"] in (6, 7)
    ret.accFaulted = self.update_acc_fault(accFaulted, parking_brake=ret.parkingBrake, drive_mode=drive_mode)

    if self.CP.flags & VolkswagenFlags.MQB_EVO:
      self.esp_hold_confirmation = bool(pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"])
    else:
      # for hold detection: VMM_02 ESP_Hold Signal is off timing and probably wrong
      # use a motion state signal instead for now
      self.esp_hold_confirmation = pt_cp.vl["ESC_50"]["Motion_State"] == 3 # full stop
      
    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    if self.CP.pcmCruise:
      ret.cruiseState.speed = int(round(ext_cp.vl["ACC_19"]["ACC_Wunschgeschw_02"])) * CV.KPH_TO_MS
      if ret.cruiseState.speed > 90:
        ret.cruiseState.speed = 0
    
    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    # turn signal effect
    self.left_blinker_active  = bool(pt_cp.vl["Blinkmodi_02"]["BM_links"])
    self.right_blinker_active = bool(pt_cp.vl["Blinkmodi_02"]["BM_rechts"])
    # turn signal cause (see door logic same schema ["Gateway_72"]["SMLS_01_alt"] is not neccessary -> SMLS_01 seems to always work)
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(240, pt_cp.vl["SMLS_01"]["BH_Blinker_li"],
                                                                            pt_cp.vl["SMLS_01"]["BH_Blinker_re"])

    # detect button configuration
    # for latched main cruise button (or no main button present), there is probably a physical cancel button
    # existing main cruise push button is probably used as cancel button if present
    main_cruise_latching = not bool(pt_cp.vl["GRA_ACC_01"]["GRA_Typ_Hauptschalter"])
    buttons = self.CCP.BUTTONS_ALT if main_cruise_latching else self.CCP.BUTTONS
    ret.buttonEvents = self.create_button_events(pt_cp, buttons)
    
    self.gra_stock_values = pt_cp.vl["GRA_ACC_01"]
    
    # Additional safety checks performed in CarInterface.
    ret.espDisabled = bool(pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"]) # this is also true for ESC Sport mode
    ret.espActive   = bool(pt_cp.vl["ESP_21"]["ESP_Eingriff"])

    self.ea_hud_stock_values = cam_cp.vl["EA_02"]

    if self.CP.flags & VolkswagenFlags.MEB:
      ret.fuelGauge = pt_cp.vl["Motor_16"]["MO_Energieinhalt_BMS"]

    self.frame += 1
    return ret

  def update_low_speed_alert(self, v_ego: float) -> bool:
    # Low speed steer alert hysteresis logic
    if (self.CP.minSteerSpeed - 1e-3) > CarControllerParams.DEFAULT_MIN_STEER_SPEED and v_ego < (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = True
    elif v_ego > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    return self.low_speed_alert

  def parse_mlb_mqb_steering_state(self, ret, pt_cp, drive_mode=True):
    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE

    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status, drive_mode)
    return

  def update_hca_state(self, hca_status, drive_mode=True):
    # Treat FAULT as temporary for worst likely EPS recovery time, for cars without factory Lane Assist
    # DISABLED means the EPS hasn't been configured to support Lane Assist
    self.eps_init_complete = self.eps_init_complete or (hca_status in ("DISABLED", "READY", "ACTIVE") or self.frame > 600)
    perm_fault = drive_mode and hca_status == "DISABLED" or (self.eps_init_complete and hca_status == "FAULT")
    temp_fault = drive_mode and hca_status in ("REJECTED", "PREEMPTED") or not self.eps_init_complete
    return temp_fault, perm_fault
    
  def update_acc_fault(self, acc_fault, parking_brake=False, drive_mode=True, recovery_frames_max=100):
    # Ignore FAULT when not in drive mode and parked
    # do not show misleading error during ignition in parked state
    # grant a short time to recover a normal cruise state
    # activate ACC before engine start while suppressing OP fault leaves to perm car fault until next full car ignition cycle
    fault = acc_fault
    if parking_brake and not drive_mode:
      fault = False
      self.cruise_recovery_timer = self.frame
    elif self.frame - self.cruise_recovery_timer < recovery_frames_max:
      fault = False
    return fault

  @staticmethod
  def get_can_parsers(CP):
    if CP.flags & VolkswagenFlags.PQ:
      return CarState.get_can_parsers_pq(CP)
    elif CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      return CarState.get_can_parsers_meb(CP)

    # manually configure some optional and variable-rate/edge-triggered messages
    pt_messages, cam_messages = [], []

    if not CP.flags & VolkswagenFlags.MLB:
      pt_messages += [
        ("Blinkmodi_02", 1)  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ]
    if CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
      cam_messages += [
        ("HCA_01", 1),  # From R242 Driver assistance camera, 50Hz if steering/1Hz if not
      ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).cam),
    }

  @staticmethod
  def get_can_parsers_pq(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).cam),
    }
    
  @staticmethod
  def get_can_parsers_meb(CP):
    pt_messages = [
      # frequency changes too much for the CANParser to figure out
      ("Blinkmodi_02", 1),  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ("SMLS_01", 1),       # From Stalk Controls
    ]
    if CP.networkLocation == NetworkLocation.fwdCamera:
      pt_messages.append(("AWV_03", 1)) # Front Collision Detection (1 Hz when inactive, 50 Hz when active)
      
    cam_messages = []
    if CP.networkLocation == NetworkLocation.gateway:
      cam_messages.append(("AWV_03", 1)) # Front Collision Detection (1 Hz when inactive, 50 Hz when active)
      
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).cam),
    }
