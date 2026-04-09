import time
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
    self.gas_release_frame = 0

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

  def create_longitudinal_command(self, acc_state, accel, counter, frame, v_ego, gas_pressed):
    set_speed = min(max(v_ego + accel, 0) * CV.MS_TO_KPH, 400)

    # There exists a jerk after overriding with gas above the set speed.
    # This jerk exists on stock TACC as well, so we ramp up jerk to avoid this.
    # Setting both jerks to 0 seems to be the only thing that prevents the unintentional braking.
    if gas_pressed:
      self.gas_release_frame = frame

    jerk = float(np.interp(frame - self.gas_release_frame, [50, 150], [0.5, CarControllerParams.JERK_LIMIT_MAX]))

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": -jerk,
      "DAS_jerkMax": jerk,
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
