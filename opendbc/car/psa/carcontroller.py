from opendbc.car import apply_std_steer_angle_limits, make_tester_present_msg, Bus
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa import psacan
from opendbc.car.psa.psacan import *
from opendbc.car.psa.values import CarControllerParams
from opendbc.car.can_definitions import CanData

# TODO: do this in interface.py init()
# Disable radar ECU by setting it to programming mode
def create_disable_radar():
  addr = 0x6B6
  bus = 1
  dat = [0x02, 0x10, 0x02, 0x80]
  dat.extend([0x0] * (8 - len(dat)))

  return CanData(addr, bytes(dat), bus)

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
    # TODO: try to use disable_ecu method, and find other method for self.frame>10
    # disable radar
    if self.radar_disabled == 0 and self.frame>10000: # TODO set to 10
      can_sends.append(create_disable_radar())
      self.radar_disabled = 1


    if self.frame % 100 == 0 and self.frame>10000: # TODO: remove 10000
      can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

    # ECU Signals that are disabled when ARTIV (radar) is inactive
    # HS2_SUPV_ARTIV_796 (ARTIV, 1 Hz, bus 1)
    # HS2_DAT_ARTIV_V2_4F6 (ARTIV, 8-10 Hz, bus 1)
    # HS2_DYN1_MDD_ETAT_2B6 (ARTIV, 50 Hz, bus 1)
    # HS2_DYN_MDD_ETAT_2F6 (ARTIV, 50 Hz, bus 1)

    # if self.CP.openpilotLongitudinalControl:
    #   if self.frame % 2: # 50 Hz
    #     can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(self.packer, self.frame // 2, actuators.accel, self.CP.openpilotLongitudinalControl))
    #     can_sends.append(create_HS2_DYN_MDD_ETAT_2F6(self.packer, self.frame // 2, actuators.accel, self.CP.openpilotLongitudinalControl))
    #   if self.frame % 10: # 10 Hz
    #     can_sends.append(create_HS2_DAT_ARTIV_V2_4F6(self.packer, self.frame, actuators.accel, self.CP.openpilotLongitudinalControl))
    #   if self.frame % 100: # 1 Hz
    #     can_sends.append(create_HS2_SUPV_ARTIV_796(self.packer, self.frame, actuators.accel, self.CP.openpilotLongitudinalControl))


    # if CC.cruiseControl.cancel:
    #   can_sends.append(create_cancel_acc(self.packer, self.frame, CS.acc_status_msg, CC.cruiseControl.cancel))

    # if CC.cruiseControl.resume:
    #   can_sends.append(create_resume_acc(self.packer, self.frame, CS.adas_status_msg, CC.cruiseControl.resume))

    ### cruise buttons ###
    # TODO: find cruise buttons msg
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
