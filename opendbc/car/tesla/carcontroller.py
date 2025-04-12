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
                                   lat_active: bool, CP, limits: AngleSteeringLimits) -> float:

  # this function applies ISO jerk and acceleration limits to the steering angle using a
  # simplistic vehicle model distilled from opendbc/car/vehicle_model.py
  # TODO I wonder if we could use the vehicle model without too much extra complexity in safety

  # *** ISO lateral jerk limit ***
  max_curvature_rate_sec = ISO_LATERAL_JERK / (max(v_ego_raw, 1) ** 2)
  max_angle_rate_sec = math.degrees(max_curvature_rate_sec * CP.steerRatio * CP.wheelbase)
  max_angle_delta = max_angle_rate_sec * (DT_CTRL * CarControllerParams.STEER_STEP)

  # limit angle delta to 5 degrees per 20ms frame to avoid faulting EPS at lower speeds
  # TODO: check stock FSD data to find the max
  max_angle_delta = min(max_angle_delta, 5.0)

  new_apply_angle = np.clip(apply_angle, apply_angle_last - max_angle_delta, apply_angle_last + max_angle_delta)

  # *** ISO lateral accel limit ***
  # TODO: add curvature factor from VM. the lack of it loses us 60% of torque at 70 m/s (1.8 m/s^2 instead of 3 m/s^2)
  curvature_accel_limit = ISO_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
  angle_accel_limit = curvature_accel_limit * CP.steerRatio * CP.wheelbase
  new_apply_angle = float(np.clip(new_apply_angle, -angle_accel_limit, angle_accel_limit))

  # angle is current steering wheel angle when inactive on all angle cars
  # TODO: should this before max lat accel limit?
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
                                                             CS.out.steeringAngleDeg, CC.latActive, self.CP,
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
