from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams, TeslaFlags


def get_steer_ctrl_type(flags: int, ctrl_type: int) -> int:
  # Legacy firmware uses a 2-bit DAS_steeringControlType, one bit up from the 3-bit signal.
  # Prevents accidentally sending LANE_KEEP_ASSIST which allows user overriding, but can be unstable and jerks back to target
  if flags & TeslaFlags.DAS_STEERING_3_BIT:
    return ctrl_type
  return ctrl_type << 1


class TeslaCAN:
  def __init__(self, CP, packer):
    self.CP = CP
    self.packer = packer

  def create_steering_control(self, angle, enabled):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": get_steer_ctrl_type(self.CP.flags, 1 if enabled else 0),  # ANGLE_CONTROL
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    set_speed = min(max(v_ego + accel, 0) * CV.MS_TO_KPH, 400)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self):
    values = {
      "APS_eacAllow": 1,
    }

    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)


def tesla_checksum(address: int, sig, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF
