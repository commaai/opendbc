import numpy as np
from opendbc.car import apply_std_steer_angle_limits, make_tester_present_msg, apply_hysteresis, Bus
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa import psacan
from opendbc.car.psa.psacan import *
from opendbc.car.psa.values import CarControllerParams

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    self.CP = CP
    self.packer = CANPacker(dbc_names[Bus.cam])
    self.frame = 0
    self.apply_angle_last = 0
    self.radar_disabled = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    ### lateral control ###
    if CC.latActive:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                   CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
    else:
      apply_angle = 0
    # TODO: check if it works without self.frame // 5
    can_sends.append(psacan.create_lka_steering(self.packer, self.frame // 5, CC.latActive, apply_angle))

    self.apply_angle_last = apply_angle

    ### longitudinal control ###
    # TUNING
    brake_accel = -0.5 # below this accel, go into brake mode
    torque_raw = actuators.accel * 10 * 80 # accel in m/s^2 to torque in Nm * 10 for CAN
    torque = max(-300, min(torque_raw, 2000)) # apply torque CAN Nm limits
    braking = actuators.accel<brake_accel and not CS.out.gasPressed

    # TODO: only enable section if self.CP.openpilotLongitudinalControl
    # TODO: disable_ecu not working - UDS communication control not supported by radar ECU.
    # disable radar ECU by setting to programming mode
    if self.frame > 1000:
      if self.radar_disabled == 0:
        can_sends.append(create_disable_radar())
        self.radar_disabled = 1

      # keep radar ECU disabled by sending tester present
      if self.frame % 100 == 0:
        can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

      # TODO: tune torque multiplier
      # TODO: tune braking threshold
      # TODO: check if disengage on accelerator is already in CC.longActive
      # Highest torque seen without gas input: ~1000
      # Lowest torque seen without break mode: -560 (but only when transitioning from brake to accel mode, else -248)
      # Lowest brake mode accel seen: -4.85m/s²

      if self.frame % 2 == 0: # 50 Hz
        can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(self.packer, self.frame // 2, actuators.accel, CC.longActive, CS.out.gasPressed, braking, torque))
        can_sends.append(create_HS2_DYN_MDD_ETAT_2F6(self.packer, self.frame // 2, CC.longActive, braking))

      if self.frame % 10 == 0: # 10 Hz
        can_sends.append(create_HS2_DAT_ARTIV_V2_4F6(self.packer, CC.longActive))

      if self.frame % 100 == 0: # 1 Hz
        can_sends.append(create_HS2_SUPV_ARTIV_796(self.packer))

    # TODO test
    # if CC.cruiseControl.cancel:
    #   can_sends.append(create_cancel_acc(self.packer, self.frame, CS.acc_status_msg, CC.cruiseControl.cancel))

    # TODO test
    # if CC.cruiseControl.resume:
    #   can_sends.append(create_resume_acc(self.packer, self.frame, CS.adas_status_msg, CC.cruiseControl.resume))

    ### cruise buttons ###
    # TODO: find cruise buttons msg
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    # TODO: logging the internal parameters for DEBUG
    new_actuators.gas = torque/1000
    new_actuators.brake = float(braking)
    new_actuators.accel = torque
    self.frame += 1
    return new_actuators, can_sends
