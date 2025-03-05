from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_lka_steering, create_longitudinal, create_wheel_touch, create_adas_status
from opendbc.car.rivian.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.last_cancel_frame = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    apply_torque = 0
    if CC.latActive:
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                     CS.out.steeringTorque, CarControllerParams)

    # send steering command
    self.apply_torque_last = apply_torque
    can_sends.append(create_lka_steering(self.packer, CS.acm_lka_hba_cmd, apply_torque, CC.latActive))

    if self.frame % 5 == 0:
      can_sends.append(create_wheel_touch(self.packer, CS.sccm_wheel_touch, CC.enabled))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      can_sends.append(create_longitudinal(self.packer, self.frame % 15, actuators.accel, CC.enabled))
    else:
      if CC.cruiseControl.cancel:
        # if (self.frame - self.last_cancel_frame) * DT_CTRL > 0.25:
        #   # send the next expected counter
        #   counter = (CS.acm_longitudinal_request["ACM_longitudinalRequest_Counter"] + 1) % 15
        #   can_sends.append(create_longitudinal(self.packer, counter, 0.0, False, True))
        #   self.last_cancel_frame = self.frame

        if (self.frame - self.last_cancel_frame) * DT_CTRL > 0.02:
          # send the next expected counter
          offset = 0
          # for offset in range(15):
          counter = (CS.vdm_adas_status["VDM_AdasStatus_Counter"] + 1 + offset) % 15
          can_sends.append(create_adas_status(self.packer, counter, CS.vdm_adas_status))
          self.last_cancel_frame = self.frame

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
