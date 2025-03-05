from opendbc.car.common.conversions import Conversions as CV
import numpy as np
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCAN:
  def __init__(self, packer):
    self.packer = packer

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

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, v_ego, active):
    set_speed = v_ego * CV.MS_TO_KPH
    accel_max, accel_min = 0, 0
    if active:
      # TODO: does this just tell the ECU which accel limit to use?
      set_speed = 0 if accel < 0 else V_CRUISE_MAX
      # set_speed = v_ego + 1 if accel > 0 else v_ego - 1
      # # in a PJ stock route, adding the min/max accel limit to vehicle speed depending on sign of v_ego - setspeed
      # # closely matched the setspeed, so we mimic that here
      # EDIT: in an FSD route it looks much less similar
      # set_speed = v_ego + accel

      # TODO: reasonable cruise limits or max actuation?
      accel_max = accel if accel >= 0 else CarControllerParams.ACCEL_MAX
      accel_min = accel if accel <= 0 else CarControllerParams.ACCEL_MIN

    values = {
      # TODO: this causes jerking after gas override when above set speed
      #"DAS_setSpeed": 0 if (accel < 0 or not enabled) else V_CRUISE_MAX,
      "DAS_setSpeed": set_speed,  # * CV.MS_TO_KPH,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      #"DAS_accelMin": accel,
      #"DAS_accelMax": max(accel, 0),
      # TODO FSD sets to ~1-2 m/s^2 when not using (active and inactive), wonder why
      # EDIT: I think it may be the max cruise +=accel at that speed. it just has a side effect (or intended behavior) of not jerking when overridden
      "DAS_accelMax": accel_max,
      "DAS_accelMin": accel_min,
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacAllow": 1,
      "APS_eacMonitorCounter": counter,
    }

    data = self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)
