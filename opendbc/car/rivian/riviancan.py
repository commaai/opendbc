def checksum(data, poly, xor_output):
  crc = 0
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = (crc << 1) ^ poly
      else:
        crc <<= 1
      crc &= 0xFF
  return crc ^ xor_output


def create_lka_steering(packer, acm_lka_hba_cmd, apply_steer, enabled):
  values = {s: acm_lka_hba_cmd[s] for s in [
    "ACM_lkaHbaCmd_Counter",
    "ACM_lkaHbaCmd_Checksum",
    "ACM_HapticRequest",
    "ACM_lkaStrToqReq",
    "ACM_lkaSymbolState",
    "ACM_lkaToiFlt",
    "ACM_lkaActToi",
    "ACM_hbaSysState",
    "ACM_FailinfoAeb",
    "ACM_lkaRHWarning",
    "ACM_lkaLHWarning",
    "ACM_lkaLaneRecogState",
    "ACM_hbaOpt",
    "ACM_hbaLamp",
    "ACM_lkaHandsoffSoundWarning",
    "ACM_lkaHandsoffDisplayWarning",
    "ACM_unkown1",
    "ACM_unkown2",
    "ACM_unkown3",
    "ACM_unkown4",
    "ACM_unkown6",
  ]}

  if enabled:
    values["ACM_lkaActToi"] = 1
    values["ACM_lkaSymbolState"] = 3
    values["ACM_lkaLaneRecogState"] = 3
    values["ACM_lkaStrToqReq"] = apply_steer
    values["ACM_unkown2"] = 1
    values["ACM_unkown3"] = 4
    values["ACM_unkown4"] = 160
    values["ACM_unkown6"] = 1

  data = packer.make_can_msg("ACM_lkaHbaCmd", 0, values)[1]
  values["ACM_lkaHbaCmd_Checksum"] = checksum(data[1:], 0x1D, 0x63)
  return packer.make_can_msg("ACM_lkaHbaCmd", 0, values)


def create_longitudinal(packer, frame, accel, enabled):
  values = {
    "ACM_longitudinalRequest_Counter": frame % 15,
    "ACM_AccelerationRequest": accel if enabled else 0,
    "ACM_VehicleHoldRequired": 0,
    "ACM_PrndRequired": 0,
    "ACM_longInterfaceEnable": 1 if enabled else 0,
    "ACM_AccelerationRequestType": 0,
  }

  data = packer.make_can_msg("ACM_longitudinalRequest", 0, values)[1]
  values["ACM_longitudinalRequest_Checksum"] = checksum(data[1:], 0x1D, 0x12)
  return packer.make_can_msg("ACM_longitudinalRequest", 0, values)

#################################################################
######################### ↓ NOT USED ↓ ##########################
#################################################################

def create_epas_system_status(packer, epas_system_status_cmd, enabled):
  values = {s: epas_system_status_cmd[s] for s in [
    "EPAS_SytemStatus_Checksum",
    "EPAS_SystemStatus_Counter",
    "EPAS_SteeringReduced",
    "EPAS_SteeringFault",
    "EPAS_SteeringMode",
    "EPAS_TorsionBarTorque",
    "EPAS_StcFault",
    "EPAS_StcActive",
    "EPAS_StcUnavailable",
    "H_CAN_EPSS_ToiFlt",
    "H_CAN_EPSS_ToiActive",
    "H_CAN_EPS_ToiUnavailable",
    "EPAS_HandsOnLevel"
  ]}

  if enabled:
    values["EPAS_HandsOnLevel"] = 1

  data = packer.make_can_msg("EPAS_SystemStatus", 2, values)[1]
  values["EPAS_SytemStatus_Checksum"] = checksum(data[1:], 0x1D, 0x1E)
  return packer.make_can_msg("EPAS_SystemStatus", 2, values)


def create_angle_steering(packer, frame, angle, active):
  values = {
    "ACM_SteeringControl_Counter": frame % 15,
    "ACM_EacEnabled": 1 if active else 0,
    "ACM_HapticRequired": 0,
    "ACM_SteeringAngleRequest": angle,
  }

  data = packer.make_can_msg("ACM_SteeringControl", 0, values)[1]
  values["ACM_SteeringControl_Checksum"] = checksum(data[1:], 0x1D, 0x41)
  return packer.make_can_msg("ACM_SteeringControl", 0, values)


def create_acm_status(packer, acm_status, active):
  values = {s: acm_status[s] for s in [
    "ACM_Status_Checksum",
    "ACM_Status_Counter",
    "ACM_Unkown1",
    "ACM_FeatureStatus",
    "ACM_FaultStatus",
    "ACM_Unkown2"
  ]}

  if active:
    values["ACM_Status_Checksum"] = (int(values["ACM_Status_Checksum"]) + 2) % 15
    values["ACM_Unkown1"] = 1

  data = packer.make_can_msg("ACM_Status", 1, values)[1]
  values["ACM_Status_Checksum"] = checksum(data[1:], 0x1D, 0x5F)
  return packer.make_can_msg("ACM_Status", 1, values)


def create_vdm_adas_status(packer, vdm_adas_status, acc_on):
  values = {s: vdm_adas_status[s] for s in [
    "VDM_AdasStatus_Checksum",
    "VDM_AdasStatus_Counter",
    "VDM_AdasDecelLimit",
    "VDM_AdasDriverAccelPriorityStatu",
    "VDM_AdasFaultStatus",
    "VDM_AdasAccelLimit",
    "VDM_AdasDriverModeStatus",
    "VDM_AdasAccelRequest",
    "VDM_AdasInterfaceStatus",
    "VDM_AdasAccelRequestAcknowledged",
    "VDM_AdasVehicleHoldStatus"
  ]}

  if acc_on:
    values["VDM_AdasDriverModeStatus"] = 1
    values["VDM_AdasAccelRequest"] = -10.16
    values["VDM_AdasInterfaceStatus"] = 1

  data = packer.make_can_msg("VDM_AdasSts", 2, values)[1]
  values["VDM_AdasStatus_Checksum"] = checksum(data[1:], 0x1D, 0xD1)
  return packer.make_can_msg("VDM_AdasSts", 2, values)
