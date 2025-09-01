import crcmod
from opendbc.car.hyundai.values import CAR, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.escc import EnhancedSmartCruiseControl
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanLeadData

hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)


def create_lkas11(packer, frame, CP, apply_torque, steer_req,
                  torque_fault, lkas11, sys_warning, sys_state, enabled,
                  left_lane, right_lane,
                  left_lane_depart, right_lane_depart,
                  lkas_icon):
  values = {s: lkas11[s] for s in [
    "CF_Lkas_LdwsActivemode",
    "CF_Lkas_LdwsSysState",
    "CF_Lkas_SysWarning",
    "CF_Lkas_LdwsLHWarning",
    "CF_Lkas_LdwsRHWarning",
    "CF_Lkas_HbaLamp",
    "CF_Lkas_FcwBasReq",
    "CF_Lkas_HbaSysState",
    "CF_Lkas_FcwOpt",
    "CF_Lkas_HbaOpt",
    "CF_Lkas_FcwSysState",
    "CF_Lkas_FcwCollisionWarning",
    "CF_Lkas_FusionState",
    "CF_Lkas_FcwOpt_USM",
    "CF_Lkas_LdwsOpt_USM",
  ]}
  values["CF_Lkas_LdwsSysState"] = sys_state
  values["CF_Lkas_SysWarning"] = 3 if sys_warning else 0
  values["CF_Lkas_LdwsLHWarning"] = left_lane_depart
  values["CF_Lkas_LdwsRHWarning"] = right_lane_depart
  values["CR_Lkas_StrToqReq"] = apply_torque
  values["CF_Lkas_ActToi"] = steer_req
  values["CF_Lkas_ToiFlt"] = torque_fault  # seems to allow actuation on CR_Lkas_StrToqReq
  values["CF_Lkas_MsgCount"] = frame % 0x10

  if CP.carFingerprint in (CAR.HYUNDAI_SONATA, CAR.HYUNDAI_PALISADE, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021, CAR.KIA_NIRO_PHEV_2022, CAR.HYUNDAI_SANTA_FE,
                           CAR.HYUNDAI_IONIQ_EV_2020, CAR.HYUNDAI_IONIQ_PHEV, CAR.KIA_SELTOS, CAR.HYUNDAI_ELANTRA_2021, CAR.GENESIS_G70_2020,
                           CAR.HYUNDAI_ELANTRA_HEV_2021, CAR.HYUNDAI_SONATA_HYBRID, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_KONA_HEV, CAR.HYUNDAI_KONA_EV_2022,
                           CAR.HYUNDAI_SANTA_FE_2022, CAR.KIA_K5_2021, CAR.HYUNDAI_IONIQ_HEV_2022, CAR.HYUNDAI_SANTA_FE_HEV_2022,
                           CAR.HYUNDAI_SANTA_FE_PHEV_2022, CAR.KIA_STINGER_2022, CAR.KIA_K5_HEV_2020, CAR.KIA_CEED,
                           CAR.HYUNDAI_AZERA_6TH_GEN, CAR.HYUNDAI_AZERA_HEV_6TH_GEN, CAR.HYUNDAI_CUSTIN_1ST_GEN, CAR.HYUNDAI_KONA_2022,
                           CAR.KIA_CEED_PHEV_2022_NON_SCC, CAR.HYUNDAI_KONA_EV_NON_SCC, CAR.HYUNDAI_ELANTRA_2022_NON_SCC,
                           CAR.GENESIS_G70_2021_NON_SCC, CAR.KIA_SELTOS_2023_NON_SCC, CAR.HYUNDAI_BAYON_1ST_GEN_NON_SCC):
    values["CF_Lkas_LdwsActivemode"] = int(left_lane) + (int(right_lane) << 1)
    values["CF_Lkas_LdwsOpt_USM"] = 2

    # FcwOpt_USM 5 = Orange blinking car + lanes
    # FcwOpt_USM 4 = Orange car + lanes
    # FcwOpt_USM 3 = Green blinking car + lanes
    # FcwOpt_USM 2 = Green car + lanes
    # FcwOpt_USM 1 = White car + lanes
    # FcwOpt_USM 0 = No car + lanes
    values["CF_Lkas_FcwOpt_USM"] = lkas_icon

    # SysWarning 4 = keep hands on wheel
    # SysWarning 5 = keep hands on wheel (red)
    # SysWarning 6 = keep hands on wheel (red) + beep
    # Note: the warning is hidden while the blinkers are on
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

  # Likely cars lacking the ability to show individual lane lines in the dash
  elif CP.carFingerprint in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.HYUNDAI_KONA_NON_SCC):
    # SysWarning 4 = keep hands on wheel + beep
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

    # SysState 0 = no icons
    # SysState 1-2 = white car + lanes
    # SysState 3 = green car + lanes, green steering wheel
    # SysState 4 = green car + lanes
    values["CF_Lkas_LdwsSysState"] = lkas_icon
    values["CF_Lkas_LdwsOpt_USM"] = 2  # non-2 changes above SysState definition

    # these have no effect
    values["CF_Lkas_LdwsActivemode"] = 0
    values["CF_Lkas_FcwOpt_USM"] = 0

  elif CP.carFingerprint == CAR.HYUNDAI_GENESIS:
    # This field is actually LdwsActivemode
    # Genesis and Optima fault when forwarding while engaged
    values["CF_Lkas_LdwsActivemode"] = 2

  dat = packer.make_can_msg("LKAS11", 0, values)[1]

  if CP.flags & HyundaiFlags.CHECKSUM_CRC8:
    # CRC Checksum as seen on 2019 Hyundai Santa Fe
    dat = dat[:6] + dat[7:8]
    checksum = hyundai_checksum(dat)
  elif CP.flags & HyundaiFlags.CHECKSUM_6B:
    # Checksum of first 6 Bytes, as seen on 2018 Kia Sorento
    checksum = sum(dat[:6]) % 256
  else:
    # Checksum of first 6 Bytes and last Byte as seen on 2018 Kia Stinger
    checksum = (sum(dat[:6]) + dat[7]) % 256

  values["CF_Lkas_Chksum"] = checksum

  return packer.make_can_msg("LKAS11", 0, values)


def create_clu11(packer, frame, clu11, button, CP):
  values = {s: clu11[s] for s in [
    "CF_Clu_CruiseSwState",
    "CF_Clu_CruiseSwMain",
    "CF_Clu_SldMainSW",
    "CF_Clu_ParityBit1",
    "CF_Clu_VanzDecimal",
    "CF_Clu_Vanz",
    "CF_Clu_SPEED_UNIT",
    "CF_Clu_DetentOut",
    "CF_Clu_RheostatLevel",
    "CF_Clu_CluInfo",
    "CF_Clu_AmpInfo",
    "CF_Clu_AliveCnt1",
  ]}
  values["CF_Clu_CruiseSwState"] = button
  values["CF_Clu_AliveCnt1"] = frame % 0x10
  # send buttons to camera on camera-scc based cars
  bus = 2 if CP.flags & HyundaiFlags.CAMERA_SCC else 0
  return packer.make_can_msg("CLU11", bus, values)


def create_lfahda_mfc(packer, enabled, lfa_icon):
  values = {
    "LFA_Icon_State": lfa_icon,
  }
  return packer.make_can_msg("LFAHDA_MFC", 0, values)


def create_acc_commands(packer, enabled, accel, upper_jerk, idx, lead_data: CanLeadData,
                        hud_control, set_speed, stopping, long_override, use_fca, CP,
                        main_cruise_enabled, tuning, ESCC: EnhancedSmartCruiseControl = None):
  commands = []

  def get_scc11_values():
    return {
      "MainMode_ACC": 1 if main_cruise_enabled else 0,
      "TauGapSet": hud_control.leadDistanceBars,
      "VSetDis": set_speed if enabled else 0,
      "AliveCounterACC": idx % 0x10,
      "ObjValid": int(lead_data.lead_visible), # close lead makes controls tighter
      "ACC_ObjStatus": int(lead_data.lead_visible), # close lead makes controls tighter
      "ACC_ObjLatPos": 0,
      "ACC_ObjRelSpd": lead_data.lead_rel_speed,
      "ACC_ObjDist": int(lead_data.lead_distance), # close lead makes controls tighter
    }

  def get_scc12_values():
    scc12_values = {
      "ACCMode": 2 if enabled and long_override else 1 if enabled else 0,
      "StopReq": 1 if tuning.stopping else 0,
      "aReqRaw": tuning.desired_accel,
      "aReqValue": tuning.actual_accel,  # stock ramps up and down respecting jerk limit until it reaches aReqRaw
      "CR_VSM_Alive": idx % 0xF,
    }

    # show AEB disabled indicator on dash with SCC12 if not sending FCA messages.
    # these signals also prevent a TCS fault on non-FCA cars with alpha longitudinal
    if not use_fca:
      scc12_values["CF_VSM_ConfMode"] = 1
      scc12_values["AEB_Status"] = 1 # AEB disabled

    # Since we have ESCC available, we can update SCC12 with ESCC values.
    if ESCC and ESCC.enabled:
      ESCC.update_scc12(scc12_values)

    return scc12_values

  def calculate_scc12_checksum(values):
    scc12_dat = packer.make_can_msg("SCC12", 0, values)[1]
    values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(i, 16)) for i in scc12_dat) % 0x10
    return values

  def get_scc14_values():
    return {
      "ComfortBandUpper": tuning.comfort_band_upper, # stock usually is 0 but sometimes uses higher values
      "ComfortBandLower": tuning.comfort_band_lower, # stock usually is 0 but sometimes uses higher values
      "JerkUpperLimit": tuning.jerk_upper, # stock usually is 1.0 but sometimes uses higher values
      "JerkLowerLimit": tuning.jerk_lower, # stock usually is 0.5 but sometimes uses higher values
      "ACCMode": 2 if enabled and long_override else 1 if enabled else 4, # stock will always be 4 instead of 0 after first disengage
      "ObjGap": lead_data.object_gap, # 5: >30, m, 4: 25-30 m, 3: 20-25 m, 2: < 20 m, 0: no lead
      "ObjDistStat": lead_data.object_rel_gap,
    }

  def get_fca11_values():
    return {
      "CR_FCA_Alive": idx % 0xF,
      "PAINT1_Status": 1,
      "FCA_DrvSetStatus": 1,
      "FCA_Status": 1,
    }

  def calculate_fca11_checksum(values):
    fca11_dat = packer.make_can_msg("FCA11", 0, values)[1]
    values["CR_FCA_ChkSum"] = hyundai_checksum(fca11_dat[:7])
    return values

  scc11_values = get_scc11_values()
  commands.append(packer.make_can_msg("SCC11", 0, scc11_values))

  scc12_values = get_scc12_values()
  scc12_values = calculate_scc12_checksum(scc12_values)
  commands.append(packer.make_can_msg("SCC12", 0, scc12_values))

  scc14_values = get_scc14_values()
  commands.append(packer.make_can_msg("SCC14", 0, scc14_values))

  # Only send FCA11 on cars where it exists on the bus
  # On Camera SCC cars, FCA11 is not disabled, so we forward stock FCA11 back to the car forward hooks
  # If we don't use ESCC since ESCC does not block FCA11 from stock radar
  if use_fca and not ((CP.flags & HyundaiFlags.CAMERA_SCC) or (ESCC and ESCC.enabled)):
    # note that some vehicles most likely have an alternate checksum/counter definition
    # https://github.com/commaai/opendbc/commit/9ddcdb22c4929baf310295e832668e6e7fcfa602
    fca11_values = get_fca11_values()
    fca11_values = calculate_fca11_checksum(fca11_values)
    commands.append(packer.make_can_msg("FCA11", 0, fca11_values))

  return commands


def create_acc_opt(packer, CP, ESCC: EnhancedSmartCruiseControl = None):
  """
    Creates SCC13 and FCA12. If ESCC is enabled, it will only create SCC13 since ESCC does not block FCA12.
    :param packer:
    :param ESCC:
    :return:
  """

  def get_scc13_values():
    return {
      "SCCDrvModeRValue": 2,
      "SCC_Equip": 1,
      "Lead_Veh_Dep_Alert_USM": 2,
    }

  def get_fca12_values():
    return {
      "FCA_DrvSetState": 2,
      "FCA_USM": 1, # AEB disabled
    }

  commands = []

  scc13_values = get_scc13_values()
  commands.append(packer.make_can_msg("SCC13", 0, scc13_values))

  # If ESCC is available and enabled, we skip FCA12, since ESCC does not block FCA12
  if ESCC and ESCC.enabled:
    return commands

  # TODO: this needs to be detected and conditionally sent on unsupported long cars
  # On Camera SCC cars, FCA12 is not disabled, so we forward stock FCA12 back to the car forward hooks
  if not (CP.flags & HyundaiFlags.CAMERA_SCC):
    fca12_values = get_fca12_values()
    commands.append(packer.make_can_msg("FCA12", 0, fca12_values))

  return commands


def create_frt_radar_opt(packer):
  frt_radar11_values = {
    "CF_FCA_Equip_Front_Radar": 1,
  }
  return packer.make_can_msg("FRT_RADAR11", 0, frt_radar11_values)
