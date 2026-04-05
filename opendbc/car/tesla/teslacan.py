import numpy as np
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams, TeslaFlags


def get_steer_ctrl_type(flags: int, ctrl_type: int) -> int:
  # Returns the flipped signal value for DAS_steeringControlType on FSD 14
  if flags & TeslaFlags.FSD_14:
    return {1: 2, 2: 1}.get(ctrl_type, ctrl_type)
  else:
    return ctrl_type


class TeslaCAN:
  def __init__(self, CP, packer):
    self.CP = CP
    self.packer = packer

  def create_steering_control(self, angle, enabled):
    # On FSD 14+, ANGLE_CONTROL behavior changed to allow user winddown while actuating.
    # with openpilot, after overriding w/ ANGLE_CONTROL the wheel snaps back to the original angle abruptly
    # so we now use LANE_KEEP_ASSIST to match stock FSD.
    # see carstate.py for more details
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": get_steer_ctrl_type(self.CP.flags, 1 if enabled else 0),
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, a_ego, active):
    # DAS_setSpeed is theorized to just be a signal for DI to determine if it should actuate accelMin or accelMax
    accel = accel if active else a_ego
    accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    set_speed = v_ego + accel * 0.7  # TODO: 0.7 may not be important
    set_speed = min(max(set_speed * CV.MS_TO_KPH, 0), 400)  # signal max is 409.4

    # while braking, accel max is positive and vice versa. it has been seen changing based on speed.
    # it may be a comfort limit in case setSpeed changes rapidly, not sure how this is actually used
    accel_min_inactive = float(np.interp(v_ego, [5, 35], [-1.56, -0.8]))
    accel_max_inactive = float(np.interp(v_ego, [0, 35], [2.0, 0.64]))

    accel_min = accel if accel <= 0 else accel_min_inactive
    accel_max = accel if accel > 0 else accel_max_inactive

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel_min,
      "DAS_accelMax": accel_max,
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
