from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams

ACCEL_WINDOW = 0.08  # m/s^2, two steps of the signal
STANDSTILL_SPEED = 0.1  # m/s
STANDSTILL_ACCEL_MIN = -0.5  # m/s^2, stock holds with about -0.5


class TeslaCAN:
  def __init__(self, CP, packer):
    self.CP = CP
    self.packer = packer

  def create_steering_control(self, angle, enabled):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,  # ANGLE_CONTROL
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    set_speed = min(max(v_ego + accel, 0) * CV.MS_TO_KPH, 400)
    accel_min = min(accel, -ACCEL_WINDOW)
    accel_max = max(accel, ACCEL_WINDOW)
    if v_ego < STANDSTILL_SPEED and accel <= 0:
      # stock holds a stop with a small accel min and a big accel max, the set speed of 0 keeps the car stopped.
      # a hard accel min while stopped faults cruise when the driver steers hard
      accel_min = max(accel_min, STANDSTILL_ACCEL_MIN)
      accel_max = CarControllerParams.ACCEL_MAX

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      # stock keeps the accel window across zero. with a closed window the car faults cruise when the driver steers hard
      "DAS_accelMin": accel_min if active else accel,
      "DAS_accelMax": accel_max if active else max(accel, 0),
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
