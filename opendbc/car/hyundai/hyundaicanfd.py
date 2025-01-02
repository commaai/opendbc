from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.numpy_fast import clip
from opendbc.car.hyundai.values import HyundaiFlags


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, hda2=None) -> None:
    super().__init__(CP, fingerprint)

    if hda2 is None:
      hda2 = CP.flags & HyundaiFlags.CANFD_HDA2.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. HDA2 cars
    # have a different harness than the HDA1 and non-HDA variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if hda2:
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam


def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_steer):

  ret = []

  values = {
    "LKA_MODE": 2,
    "LKA_ICON": 2 if enabled else 1,
    "TORQUE_REQUEST": apply_steer,
    "LKA_ASSIST": 0,
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_MODE": 0,
    "HAS_LANE_SAFETY": 0,  # hide LKAS settings
    "NEW_SIGNAL_1": 0,
    "NEW_SIGNAL_2": 0,
  }

  if CP.flags & HyundaiFlags.CANFD_HDA2:
    hda2_lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
    ret.append(packer.make_can_msg(hda2_lkas_msg, CAN.ACAN, values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret

def create_suppress_lfa(packer, CAN, hda2_lfa_block_msg, hda2_alt_steering):
  suppress_msg = "CAM_0x362" if hda2_alt_steering else "CAM_0x2a4"
  msg_bytes = 32 if hda2_alt_steering else 24

  values = {f"BYTE{i}": hda2_lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values["COUNTER"] = hda2_lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0
  values["RIGHT_LANE_LINE"] = 0
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)

def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_HDA2 else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)

def create_acc_cancel(packer, CP, CAN, cruise_info_copy):
  # TODO: why do we copy different values here?
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "NEW_SIGNAL_1",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "ZEROS_5",
      "DISTANCE_SETTING",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)

def create_lfahda_cluster(packer, CAN, enabled):
  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": 2 if enabled else 0,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)

def create_msg_161(packer, CAN, enabled, msg_161, car_params, hud_control, car_state, car_control, frame):
  values = msg_161.copy()

  # HIDE ALERTS
  if values.get("ALERTS_5") == 5:  # USE SWITCH OR PEDAL TO ACCELERATE
    values["ALERTS_5"] = 0
  if values.get("ALERTS_2") == 5:  # CONSIDER TAKING A BREAK
    values.update({"ALERTS_2": 0, "SOUNDS_2": 0, "DAW_ICON": 0})

  # LANELINES
  curvature = {
    i: (31 if i == -1 else 13 - abs(i + 15)) if i < 0 else 15 + i
    for i in range(-15, 16)
  }
  values.update({
    "LANELINE_CURVATURE": curvature.get(max(-15, min(int(car_state.out.steeringAngleDeg / 3), 15)), 14) if enabled else 15,
    "LFA_ICON": 2 if enabled else 0,
    "LKA_ICON": 4 if enabled else 0,
    "LANELINE_LEFT": 2 if enabled else 0,
    "LANELINE_RIGHT": 2 if enabled else 0,
    "CENTERLINE": 1 if enabled else 0,
  })

  # LCA
  if enabled:
    speed_below_threshold = car_state.out.vEgo < 8.94
    values.update({
      "LCA_LEFT_ICON": 0 if car_state.out.leftBlindspot or speed_below_threshold else 2 if car_control.leftBlinker else 1,
      "LCA_RIGHT_ICON": 0 if car_state.out.rightBlindspot or speed_below_threshold else 2 if car_control.rightBlinker else 1,
      "LCA_LEFT_ARROW": 2 if car_control.leftBlinker else 0,
      "LCA_RIGHT_ARROW": 2 if car_control.rightBlinker else 0,
    })

  # LANE DEPARTURE
  if hud_control.leftLaneDepart:
    values["LANELINE_LEFT"] = 4 if (frame // 50) % 2 == 0 else 1
  if hud_control.rightLaneDepart:
    values["LANELINE_RIGHT"] = 4 if (frame // 50) % 2 == 0 else 1

  if car_params.openpilotLongitudinalControl:
    # HIDE ALERTS
    if values.get("ALERTS_5") == 4:  # SMART CRUISE CONTROL CONDITIONS NOT MET
      values["ALERTS_5"] = 0

    # SETSPEED
    values["SETSPEED"] = 3 if enabled else 1
    values["SETSPEED_HUD"] = 2 if enabled else 1
    values["SETSPEED_SPEED"] = 25 if (s := round(car_state.out.vCruiseCluster * CV.KPH_TO_MPH)) > 100 else s

    # DISTANCE
    if 1 <= hud_control.leadDistanceBars <= 3:
      values["DISTANCE"] = hud_control.leadDistanceBars
      values["DISTANCE_SPACING"] = 1 if enabled else 0
      values["DISTANCE_LEAD"] = 2 if enabled and hud_control.leadVisible else 1 if enabled else 0
      values["DISTANCE_CAR"] = 2 if enabled else 1
      values["ALERTS_3"] = hud_control.leadDistanceBars + 6
    else:
      values["DISTANCE"] = 0
      values["DISTANCE_SPACING"] = 0
      values["DISTANCE_LEAD"] = 0
      values["DISTANCE_CAR"] = 0

    # BACKGROUND
    values["BACKGROUND"] = 1 if enabled else 7

  return packer.make_can_msg("MSG_161", CAN.ECAN, values)

def create_msg_162(packer, CAN, enabled, msg_162, car_params, hud_control):
  values = msg_162.copy()

  # HIDE FAULTS
  values.update({
    "FAULT_LSS": 0,
    "FAULT_HDA": 0,
    "FAULT_DAS": 0,
  })

  # LANE DEPARTURE
  if hud_control.leftLaneDepart or hud_control.rightLaneDepart:
    values["VIBRATE"] = 1

  if car_params.openpilotLongitudinalControl:
    # *** TODO *** LEAD_DISTANCE/LEAD_LATERAL
    # LEAD
    if hud_control.leadVisible:
      values["LEAD"] = 2 if enabled else 1
      values["LEAD_DISTANCE"] = 100
    else:
      values["LEAD"] = 0
      values["LEAD_DISTANCE"] = 0

  return packer.make_can_msg("MSG_162", CAN.ECAN, values)

def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control):
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = clip(accel, accel_last - jn, accel_last + jn)

  values = {
    "ACCMode": 0 if not enabled else (2 if gas_override else 1),
    "MainMode_ACC": 1,
    "StopReq": 1 if stopping else 0,
    "aReqValue": a_val,
    "aReqRaw": a_raw,
    "VSetDis": set_speed,
    "JerkLowerLimit": jerk if enabled else 1,
    "JerkUpperLimit": 3.0,

    "ACC_ObjDist": 1,
    "ObjValid": 0,
    "OBJ_STATUS": 2,
    "SET_ME_2": 0x4,
    "SET_ME_3": 0x3,
    "SET_ME_TMP_64": 0x64,
    "DISTANCE_SETTING": hud_control.leadDistanceBars,
  }

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_spas_messages(packer, CAN, frame, left_blink, right_blink):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_adrv_messages(packer, CAN, frame):
  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("ADRV_0x51", CAN.ACAN, values))

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x2,
      'SET_ME_FF': 0xff,
      'SET_ME_FC': 0xfc,
      'SET_ME_9': 0x9,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))

  if frame % 5 == 0:
    values = {
      'SET_ME_1C': 0x1c,
      'SET_ME_FF': 0xff,
      'SET_ME_TMP_F': 0xf,
      'SET_ME_TMP_F_2': 0xf,
    }
    ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

    values = {
      'SET_ME_E1': 0xe1,
      'SET_ME_3A': 0x3a,
    }
    ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

  if frame % 20 == 0:
    values = {
      'SET_ME_15': 0x15,
    }
    ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

  if frame % 100 == 0:
    values = {
      'SET_ME_22': 0x22,
      'SET_ME_41': 0x41,
    }
    ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

  return ret
