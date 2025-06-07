def calculate_checksum(dat: bytearray, chk_ini: int) -> int:
  checksum = sum((b >> 4) + (b & 0xF) for b in dat)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, frame: int, lat_active: bool, apply_angle: float):
  values = {
    'DRIVE': 1,
    'COUNTER': frame % 0x10,
    'CHECKSUM': 0,
    # Cycle STATUS 2->3->4->2.. this keeps control active. 0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
    'STATUS': (frame % 3) + 2 if lat_active else 0,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1]
  values['CHECKSUM'] = calculate_checksum(msg, 0xB)

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


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
