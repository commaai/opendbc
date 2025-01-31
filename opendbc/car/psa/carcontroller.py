from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.psa import psacan
from openpilot.selfdrive.car.psa.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    self.lkas_max_torque = 0
    self.apply_angle_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators

    ### lateral control ###
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        # windup slower
        apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CarControllerParams)
        apply_angle = clip(apply_angle, -CarControllerParams.STEER_MAX, CarControllerParams.STEER_MAX)

        # Max torque from driver before EPS will give up and not apply torque
        if not bool(CS.out.steeringPressed):
          self.lkas_max_torque = CarControllerParams.LKAS_MAX_TORQUE
        else:
          # Scale max torque based on how much torque the driver is applying to the wheel
          self.lkas_max_torque = max(
            # Scale max torque down to half LKAX_MAX_TORQUE as a minimum
            CarControllerParams.LKAS_MAX_TORQUE * 0.5,
            # Start scaling torque at STEER_THRESHOLD
            CarControllerParams.LKAS_MAX_TORQUE - 0.6 * max(0, abs(CS.out.steeringTorque) - CarControllerParams.STEER_THRESHOLD)
          )

      else:
        apply_angle = CS.out.steeringAngleDeg
        self.lkas_max_torque = 0

      can_sends.append(psacan.create_lka_msg(self.packer, self.CP, apply_angle, self.frame, CC.latActive, self.lkas_max_torque))

      self.apply_angle_last = apply_angle

    ### cruise buttons ###
    # TODO: find cruise buttons msg

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.steer = self.lkas_max_torque

    self.frame += 1
    return new_actuators, can_sends
