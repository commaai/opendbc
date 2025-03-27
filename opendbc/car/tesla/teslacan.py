from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCAN:
  def __init__(self, dbc_names, is_3Y):
    self.is_3Y = is_3Y
    self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party])}
    if not self.is_3Y:
      self.packers[CANBUS.powertrain] = CANPacker(dbc_names[Bus.pt])

  def create_steering_control(self, angle, enabled):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    return self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      # TODO: this causes jerking after gas override when above set speed
      set_speed = 0 if accel < 0 else V_CRUISE_MAX

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

    # TODO: this might need the other checksum address (0x2b9) on Raven too
    return self.packers[CANBUS.party if self.is_3Y else CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self):
    values = {
      "APS_eacAllow": 1,
    }

    return self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)
