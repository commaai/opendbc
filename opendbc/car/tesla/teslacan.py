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

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
      "DAS_steeringControlCounter": counter,
    }

    data = self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, v_ego, active):
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
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }

    bus = CANBUS.party if self.is_3Y else CANBUS.powertrain
    data = self.packers[bus].make_can_msg("DAS_control", bus, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packers[bus].make_can_msg("DAS_control", bus, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacAllow": 1,
      "APS_eacMonitorCounter": counter,
    }

    data = self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)
