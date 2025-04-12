import numpy as np
import math
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_std_steer_angle_limits, AngleSteeringLimits, DT_CTRL
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CarControllerParams

# Note that Tesla safety supports up to ISO 11270 limits, but is comfort limited in openpilot (TODO: planner, controls?)
# TODO: copy ford's std assumed roll compensation
ISO_LATERAL_ACCEL = 3.0  # m/s^2  # TODO: import from test lateral limits file?
ISO_LATERAL_JERK = 5.0  # m/s^3


def apply_tesla_steer_angle_limits(apply_angle: float, apply_angle_last: float, v_ego_raw: float, steering_angle: float,
                                   lat_active: bool, CP, CCP, limits: AngleSteeringLimits) -> float:
  # pick angle rate limits based on wind up/down
  # TODO: use ISO_LATERAL_JERK here instead of 3 breakpoint list
  # steer_up = apply_angle_last * apply_angle >= 0. and abs(apply_angle) > abs(apply_angle_last)

  # max_accel_diff = ISO_LATERAL_JERK / (100 / CCP.STEER_STEP)
  # ISO_LATERAL_JERK / (v_ego ** 2)
  max_curvature_rate = ISO_LATERAL_JERK / (v_ego_raw ** 2)  # s
  max_angle_rate = math.degrees(max_curvature_rate * CP.steerRatio * CP.wheelbase)  # s
  max_angle_delta = max_angle_rate * (DT_CTRL * CCP.STEER_STEP)  # per frame

  # rate_limits = limits.ANGLE_RATE_LIMIT_UP if steer_up else limits.ANGLE_RATE_LIMIT_DOWN

  # angle_rate_lim = np.interp(v_ego_raw, rate_limits[0], rate_limits[1])
  new_apply_angle = np.clip(apply_angle, apply_angle_last - max_angle_delta, apply_angle_last + max_angle_delta)

  # limit max angle from max lateral accel
  # TODO: add curvature factor from VM. the lack of it loses us 60% of torque at 70 m/s (1.8 m/s^2 instead of 3 m/s^2)
  curvature_accel_limit = ISO_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
  angle_accel_limit = curvature_accel_limit * CP.steerRatio * CP.wheelbase
  new_apply_angle = float(np.clip(new_apply_angle, -angle_accel_limit, angle_accel_limit))

  # angle is current steering wheel angle when inactive on all angle cars
  if not lat_active:
    new_apply_angle = steering_angle

  return float(np.clip(new_apply_angle, -limits.STEER_ANGLE_MAX, limits.STEER_ANGLE_MAX))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Disengage and allow for user override on high torque inputs
    # TODO: move this to a generic disengageRequested carState field and set CC.cruiseControl.cancel based on it
    hands_on_fault = CS.hands_on_level >= 3
    cruise_cancel = CC.cruiseControl.cancel or hands_on_fault
    lat_active = CC.latActive and not hands_on_fault

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_tesla_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                             CS.out.steeringAngleDeg, CC.latActive, self.CP, CarControllerParams,
                                                             CarControllerParams.ANGLE_LIMITS)

      can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active, (self.frame // 2) % 16))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed((self.frame // 10) % 16))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if cruise_cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if cruise_cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
