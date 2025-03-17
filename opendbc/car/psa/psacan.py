def calculate_checksum(dat: bytearray, chk_ini: int) -> int:
  checksum = sum((b >> 4) + (b & 0xF) for b in dat)
  return (chk_ini - checksum) & 0xF


# Radar, 50 Hz
def create_HS2_DYN1_MDD_ETAT_2B6(packer, frame: int, accel: float, enabled: bool):
  values = {
    'MDD_DESIRED_DECELERATION': -10.65,
    'POTENTIAL_WHEEL_TORQUE_REQUEST': 2,
    'MIN_TIME_FOR_DESIRED_GEAR': 0.0,
    'GMP_POTENTIAL_WHEEL_TORQUE': -4000,
    'ACC_STATUS': 4,
    'GMP_WHEEL_TORQUE': -4000,
    'WHEEL_TORQUE_REQUEST': 0,
    'AUTO_BRAKING_STATUS': 3,
    'MDD_DECEL_TYPE': 1,
    'MDD_DECEL_CONTROL_REQ': 1,
    'GEAR_TYPE': 0,
    'PREFILL_REQUEST': 0,
    'DYN_ACC_CHECKSUM': 0,
    'DYN_ACC_PROCESS_COUNTER': frame % 0x10,
  }

  msg = packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)[1]
  values['DYN_ACC_CHECKSUM'] = calculate_checksum(msg, 0x3)

  return packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)


# Radar, 50 Hz
def create_HS2_DYN_MDD_ETAT_2F6(packer, frame: int, accel: float, enabled: bool):
  values = {
    'TARGET_DETECTED': 1,
    'REQUEST_TAKEOVER': 0, # TODO potential signal for HUD message from OP
    'BLIND_SENSOR': 0,
    'REQ_VISUAL_COLL_ALERT_ARC': 0,
    'REQ_AUDIO_COLL_ALERT_ARC': 0,
    'REQ_HAPTIC_COLL_ALERT_ARC': 0,
    'INTER_VEHICLE_DISTANCE': 3.5,
    'ARC_STATUS': 6,
    'AUTO_BRAKING_IN_PROGRESS': 0,
    'AEB_ENABLED': 0,
    'DRIVE_AWAY_REQUEST': 0,
    'DISPLAY_INTERVEHICLE_TIME': 6.1,
    'MDD_DECEL_CONTROL_REQ': 1,
    'AUTO_BRAKING_STATUS': 3,
    'CHECKSUM_TRANSM_DYN_ACC2': 0,
    'PROCESS_COUNTER_4B_ACC2': frame % 0x10,
    'TARGET_POSITION': 2,
  }

  msg = packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)[1]
  values['CHECKSUM_TRANSM_DYN_ACC2'] = calculate_checksum(msg, 0x8)

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)









def create_lka_steering(packer, frame: int, lat_active: bool, apply_angle: float):
  values = {
    'DRIVE': 1,
    'COUNTER': frame % 0x10,
    'CHECKSUM': 0,
    'STATUS': (frame % 3) + 2 if lat_active else 2, # Cycle status 2->3->4->2.. this keeps control active 2: READY, 3: AUTHORIZED, 4: ACTIVE
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1]
  values['CHECKSUM'] = calculate_checksum(msg, 0xB)

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


# # Radar, 50 Hz
# def create_HS2_DYN1_MDD_ETAT_2B6(packer, frame: int, accel: float, enabled: bool):
#   values = {
#     'MDD_DESIRED_DECELERATION': accel if enabled else 2.05, # m/s²
#     'POTENTIAL_WHEEL_TORQUE_REQUEST': accel if enabled else 0,
#     'MIN_TIME_FOR_DESIRED_GEAR': 0.0 if enabled else 6.2,
#     'GMP_POTENTIAL_WHEEL_TORQUE': 100 if enabled else -4000,
#     'ACC_STATUS': 4 if enabled else 3,
#     'GMP_WHEEL_TORQUE': 100 if enabled else -4000, # TODO
#     'WHEEL_TORQUE_REQUEST': 1 if enabled else 0,
#     'AUTO_BRAKING_STATUS': 6,
#     'MDD_DECEL_TYPE': 1 if accel < 0 else 0,
#     'MDD_DECEL_CONTROL_REQ': 1 if accel < 0 else 0,
#     'GEAR_TYPE': frame % 2, #0,1,0,1...
#     'PREFILL_REQUEST': 0,
#     'DYN_ACC_CHECKSUM': 0,
#     'DYN_ACC_PROCESS_COUNTER': frame % 0x10,
#   }

#   msg = packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)[1]
#   values['DYN_ACC_CHECKSUM'] = calculate_checksum(msg, 0x3)

#   return packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)


# # Radar, 50 Hz
# def create_HS2_DYN_MDD_ETAT_2F6(packer, frame: int, accel: float, enabled: bool):
#   values = {
#     'TARGET_DETECTED': 1 if enabled else 0,
#     'REQUEST_TAKEOVER': 0, # TODO potential signal for HUD message from OP
#     'BLIND_SENSOR': 0,
#     'REQ_VISUAL_COLL_ALERT_ARC': 0,
#     'REQ_AUDIO_COLL_ALERT_ARC': 0,
#     'REQ_HAPTIC_COLL_ALERT_ARC': 0,
#     'INTER_VEHICLE_DISTANCE': 100 if enabled else 255.5, # TODO
#     'ARC_STATUS': 12 if enabled else 11,
#     'AUTO_BRAKING_IN_PROGRESS': 0,
#     'AEB_ENABLED': 0,
#     'DRIVE_AWAY_REQUEST': 0, # TODO: potential RESUME request?
#     'DISPLAY_INTERVEHICLE_TIME': 3.0 if enabled else 6.2, # TODO
#     'MDD_DECEL_CONTROL_REQ': 1 if accel < 0 else 0,
#     'AUTO_BRAKING_STATUS': 6 if enabled else 3,
#     'CHECKSUM_TRANSM_DYN_ACC2': 0,
#     'PROCESS_COUNTER_4B_ACC2': frame % 0x10,
#     'TARGET_POSITION': 4,
#   }

#   msg = packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)[1]
#     values['CHECKSUM_TRANSM_DYN_ACC2'] = calculate_checksum(msg, 0x8)

#   return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)


# Radar, 10 Hz
def create_HS2_DAT_ARTIV_V2_4F6(packer, frame: int, accel: float, enabled: bool):
  values = {
    'TIME_GAP': 3.0 if enabled else 25.5, # TODO sync with 2F6
    'DISTANCE_GAP': 100 if enabled else 254, # TODO sync with 2F6
    'RELATIVE_SPEED': 0.0 if enabled else 93.8,
    'ARTIV_SENSOR_STATE': 2,
    'TARGET_DETECTED': 1 if enabled else 0,
    'ARTIV_TARGET_CHANGE_INFO': 0,
    'TRAFFIC_DIRECTION': 0, # Right hand traffic
  }
  return packer.make_can_msg('HS2_DAT_ARTIV_V2_4F6', 1, values)


# Radar, 1 Hz
def create_HS2_SUPV_ARTIV_796(packer, frame: int, accel: float, enabled: bool):
  values = {
    'FAULT_CODE': 0,
    'STATUS_NO_CONFIG': 0,
    'STATUS_PARTIAL_WAKEUP_GMP': 0,
    'UCE_ELECTR_STATE': 0,
  }
  return packer.make_can_msg('HS2_SUPV_ARTIV_796', 1, values)


def create_cancel_acc(packer, acc_status_msg, frame: int, cancel: bool):
  values = {s: acc_status_msg[s] for s in [
    'LONGITUDINAL_REGULATION_TYPE',
    'TURN_SIGNAL_STATUS',
    'FRONT_WIPER_STATUS',
    'SPEED_SETPOINT',
    'CHECKSUM_CONS_RVV_LVV2',
    'BRAKE_ONLY_CMD_BSI',
    'LVV_ACTIVATION_REQ',
    'RVV_ACC_ACTIVATION_REQ',
    'ARC_HABIT_SENSITIVITY',
    'ARC_HABIT_ACTIVATION_REQ',
    'FRAME_COUNTER_BSI2',
    'FRONT_WASH_STATUS',
    'FORCE_ACTIVATION_HAB_CMD',
    'INTER_VEHICLE_TIME_SETPOINT',
    'CHECKSUM_SPEED_SETPOINT',
    'COCKPIT_GO_ACC_REQUEST',
    'ACC_PROGRAM_MODE',
  ]}

  if cancel:
    values['RVV_ACC_ACTIVATION_REQ'] = 0

  return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, values)
  # TODO: for reference
  # checksum covers only SPEED_SETPOINT and FRAME_COUNTER_BSI2
  # 'CHECKSUM_CONS_RVV_LVV2': (((set_speed >> 4) & 1) << 1) | (set_speed & 1),
  # data = bytearray([set_speed & 0xFF, values['FRAME_COUNTER_BSI2'] & 0xFF])
  # values['CHECKSUM_SPEED_SETPOINT'] = calculate_checksum(data)


# TODO: find another message "REQUEST TAKEOVER" is likely not for "resume"
def create_resume_acc(packer, adas_status_msg, frame: int, resume: bool):
  values = {s: adas_status_msg[s] for s in [
    'TARGET_DETECTED',
    'REQUEST_TAKEOVER',
    'BLIND_SENSOR',
    'REQ_VISUAL_COLLISION_ALERT',
    'REQ_SOUND_COLLISION_ALERT',
    'REQ_HAPTIC_COLLISION_ALERT',
    'VEHICLE_INTER_DISTANCE',
    'COLLISION_ALERT_STATE',
    'AUTO_BRAKING_IN_PROGRESS',
    'AEB_ENABLED',
    'DRV_AWAY_REQ',
    'DISPLAYED_INTER_VEHICLE_TIME',
    'REQ_UCF_DECEL_CONTROL',
    'BRAKE_STATE',
    'DYN_ACC2_FRAME_CHECKSUM',
    'PROCESS_COUNTER_4B_ACC2',
    'TARGET_POSITION'
  ]}

  if resume:
    values['REQUEST_TAKEOVER'] = 1

  values['DYN_ACC2_FRAME_CHECKSUM'] = 0
  msg = packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)[1]
  values['CHECKSUM_TRANSM_DYN_ACC2'] = calculate_checksum(msg, 0x8)

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)
