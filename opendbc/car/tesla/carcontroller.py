import copy
from opendbc.car.common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from opendbc.car import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import DBC, CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_name)
    self.tesla_can = TeslaCAN(self.packer)

  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    can_sends = []

    # Disengage and allow for user override
    hands_on_fault = CS.steer_warning == "EAC_ERROR_HANDS_ON" and CS.hands_on_level >= 3
    lkas_enabled = CC.latActive and not hands_on_fault

    if self.frame % 2 == 0:
      if lkas_enabled:
        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)

        # To not fault the EPS
        apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - 20, CS.out.steeringAngleDeg + 20)
      else:
        apply_angle = CS.out.steeringAngleDeg

      self.apply_angle_last = apply_angle
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, lkas_enabled, (self.frame // 2) % 16))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl and self.frame % 4 == 0:
      state = CS.das_control["DAS_accState"] if CC.longActive else 0
      if hands_on_fault:
        state = 13 # "ACC_CANCEL_GENERIC_SILENT"
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      speed = max(CS.out.vEgo + (accel * CarControllerParams.ACCEL_TO_SPEED_MULTIPLIER), 0)
      min_accel = max(accel, 0)
      max_accel = accel
      cntr =  (self.frame // 4) % 8
      can_sends.append(self.tesla_can.create_longitudinal_command(state, speed, max_accel, min_accel, cntr))

    # Increment counter so cancel is prioritized even without openpilot longitudinal
    if hands_on_fault and not self.CP.openpilotLongitudinalControl:
      cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
      can_sends.append(self.tesla_can.create_cancel_command(cntr))

    # TODO: HUD control
    new_actuators = copy.copy(actuators)
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
