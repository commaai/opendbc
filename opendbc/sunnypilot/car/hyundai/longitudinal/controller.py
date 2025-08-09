"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from dataclasses import dataclass

from opendbc.car import structs, DT_CTRL
from opendbc.car.interfaces import CarStateBase
from opendbc.car.hyundai.values import CarControllerParams
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_car_config, jerk_limited_integrator, ramp_update
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState

MIN_JERK = 0.5
COMFORT_BAND_VAL = 0.01

DYNAMIC_LOWER_JERK_BP = [-2.0, -1.5, -1.0, -0.25, -0.1, -0.025, -0.01, -0.005]
DYNAMIC_LOWER_JERK_V  = [3.3,  2.5,  2.0,   1.9,  1.8,   1.65,  1.15,    0.5]


@dataclass
class LongitudinalState:
  desired_accel: float = 0.0
  actual_accel: float = 0.0
  accel_last: float = 0.0
  jerk_upper: float = 0.0
  jerk_lower: float = 0.0
  comfort_band_upper: float = 0.0
  comfort_band_lower: float = 0.0
  stopping: bool = False


class LongitudinalController:
  """Longitudinal controller which gets injected into CarControllerParams."""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP

    self.tuning = LongitudinalState()
    self.car_config = get_car_config(CP)
    self.long_control_state_last = LongCtrlState.off
    self.stopping_count = 0

    self.accel_cmd = 0.0
    self.desired_accel = 0.0
    self.actual_accel = 0.0
    self.accel_last = 0.0
    self.jerk_upper = 0.0
    self.jerk_lower = 0.0
    self.comfort_band_upper = 0.0
    self.comfort_band_lower = 0.0
    self.stopping = False

  @property
  def enabled(self) -> bool:
    return bool(self.CP_SP.flags & (HyundaiFlagsSP.LONG_TUNING_DYNAMIC | HyundaiFlagsSP.LONG_TUNING_PREDICTIVE))

  def get_stopping_state(self, actuators: structs.CarControl.Actuators) -> None:
    stopping = actuators.longControlState == LongCtrlState.stopping

    # If custom tuning is not enabled, use upstream stopping logic
    if not self.enabled:
      self.stopping = stopping
      self.stopping_count = 0
      return

    # Reset stopping state when not in stopping mode
    if not stopping:
      self.stopping = False
      self.stopping_count = 0
      return

    # When transitioning from off state to stopping
    if self.long_control_state_last == LongCtrlState.off:
      self.stopping = True
      return

    # Keep track of time in stopping state (in control cycles)
    if self.stopping_count > 1 / (DT_CTRL * 2):
      self.stopping = True

    self.stopping_count += 1

  @staticmethod
  def _calculate_speed_based_jerk_limits(velocity: float, long_control_state: LongCtrlState) -> tuple[float, float]:
    """Calculate jerk limits based on vehicle speed according to ISO 15622:2018.

    Args:
        velocity: Current vehicle speed (m/s)
        long_control_state: Current longitudinal control state

    Returns:
        Tuple of (upper_limit, lower_limit) in m/s³
    """

    # Upper jerk limit varies based on speed and control state
    if long_control_state == LongCtrlState.pid:
      upper_limit = float(np.interp(velocity, [0.0, 5.0, 20.0], [2.0, 3.0, 1.6]))
    else:
      upper_limit = 0.5  # Default for non-PID states

    # Lower jerk limit varies based on speed
    lower_limit = float(np.interp(velocity, [0.0, 5.0, 20.0], [5.0, 4.0, 2.5]))

    return upper_limit, lower_limit

  def _calculate_lookahead_jerk(self, accel_error: float, velocity: float) -> tuple[float, float]:
    """Calculate lookahead jerk needed to reach target acceleration.

    Args:
        accel_error: Difference between target and current acceleration (m/s²)
        velocity: Current vehicle speed (m/s)

    Returns:
        Tuple of (upper_jerk, lower_jerk) in m/s³
    """

    # Time window to reach target acceleration, varies with speed
    future_t_upper = float(np.interp(velocity, self.car_config.lookahead_jerk_bp, self.car_config.lookahead_jerk_upper_v))
    future_t_lower = float(np.interp(velocity, self.car_config.lookahead_jerk_bp, self.car_config.lookahead_jerk_lower_v))

    # Required jerk to reach target acceleration in lookahead window
    j_ego_upper = accel_error / future_t_upper
    j_ego_lower = accel_error / future_t_lower

    return j_ego_upper, j_ego_lower

  def _calculate_dynamic_lower_jerk(self, accel_error: float, velocity: float) -> float:
    """Calculate dynamic jerk for braking based on acceleration error.

    Used for the dynamic tuning approach (non-predictive).

    Args:
        accel_error: Difference between actual and previous acceleration (m/s²)
        velocity: Current vehicle speed (m/s)

    Returns:
        Dynamic lower jerk limit (m/s³)
    """

    if self.CP.radarUnavailable:
      return 5.0

    if accel_error < 0:
      # Scale the brake jerk values based on car config
      lower_max = self.car_config.jerk_limits
      original_values = np.array(DYNAMIC_LOWER_JERK_V)
      scaled_values = original_values * (lower_max / original_values[0])

      # Interpolate based on acceleration error
      dynamic_lower_jerk = float(np.interp(accel_error, DYNAMIC_LOWER_JERK_BP, scaled_values))
    else:
      dynamic_lower_jerk = 0.5

    return dynamic_lower_jerk

  def calculate_jerk(self, CC: structs.CarControl, CS: CarStateBase, long_control_state: LongCtrlState) -> None:
    """Calculate appropriate jerk limits for smooth acceleration/deceleration.

    Args:
        CC: Car control signals
        CS: Car state
        long_control_state: Current longitudinal control state
    """

    # If custom tuning is disabled, use upstream fixed values
    if not self.enabled:
      jerk_limit = 3.0 if long_control_state == LongCtrlState.pid else 1.0
      self.jerk_upper = jerk_limit
      self.jerk_lower = 5.0
      return

    velocity = CS.out.vEgo
    accel_error = self.accel_cmd - self.accel_last

    # Calculate jerk limits based on speed
    upper_speed_factor, lower_speed_factor = self._calculate_speed_based_jerk_limits(velocity, long_control_state)

    # Calculate lookahead jerk
    j_ego_upper, j_ego_lower = self._calculate_lookahead_jerk(accel_error, velocity)

    # Calculate lower jerk limit
    lower_jerk = max(-j_ego_lower, MIN_JERK)
    if self.CP.radarUnavailable:
      lower_jerk = 5.0

    # Final jerk limits with thresholds
    desired_jerk_upper = min(max(j_ego_upper, MIN_JERK), upper_speed_factor)
    desired_jerk_lower = min(lower_jerk, lower_speed_factor)

    # Calculate dynamic lower jerk for non-predictive tuning
    a_ego_blended = float(np.interp(velocity, [1.0, 2.0], [CS.aBasis, CS.out.aEgo]))
    dynamic_accel_error = a_ego_blended - self.accel_last
    dynamic_lower_jerk = self._calculate_dynamic_lower_jerk(dynamic_accel_error, velocity)
    dynamic_desired_lower_jerk = min(dynamic_lower_jerk, lower_speed_factor)

    # Apply jerk limits based on tuning approach
    self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)

    # Predictive tuning uses calculated desired jerk directly
    # Dynamic tuning applies a ramped approach for smoother transitions
    if self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING_PREDICTIVE:
      self.jerk_lower = desired_jerk_lower
    else:
      self.jerk_lower = ramp_update(self.jerk_lower, dynamic_desired_lower_jerk)

    # Disable jerk when longitudinal control is inactive
    if not CC.longActive:
      self.jerk_upper = 0.0
      self.jerk_lower = 0.0

  def calculate_accel(self, CC: structs.CarControl) -> None:
    """Calculate commanded acceleration using jerk-limited approach.

    Args:
        CC: Car control signals
    """

    # Skip custom processing if tuning is disabled or radar unavailable
    if not self.enabled or self.CP.radarUnavailable:
      self.desired_accel = self.accel_cmd
      self.actual_accel = self.accel_cmd
      return

    # Reset acceleration when control is inactive
    if not CC.longActive:
      self.desired_accel = 0.0
      self.actual_accel = 0.0
      self.accel_last = 0.0
      return

    # Force zero acceleration during stopping
    if self.stopping:
      self.desired_accel = 0.0
    else:
      self.desired_accel = float(np.clip(self.accel_cmd, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    # Apply jerk-limited integration to get smooth acceleration
    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.accel_last, self.jerk_upper, self.jerk_lower)

    self.accel_last = self.actual_accel

  def calculate_comfort_band(self, CC: structs.CarControl) -> None:
    if not self.enabled or self.CP.radarUnavailable or not CC.longActive:
      self.comfort_band_upper = 0.0
      self.comfort_band_lower = 0.0
      return

    self.comfort_band_upper = COMFORT_BAND_VAL
    self.comfort_band_lower = COMFORT_BAND_VAL

  def get_tuning_state(self) -> None:
    """Update the tuning state object with current control values.

    External components depend on this state for longitudinal control.
    """

    self.tuning = LongitudinalState(
      desired_accel=self.desired_accel,
      actual_accel=self.actual_accel,
      accel_last=self.accel_last,
      jerk_upper=self.jerk_upper,
      jerk_lower=self.jerk_lower,
      comfort_band_upper=self.comfort_band_upper,
      comfort_band_lower=self.comfort_band_lower,
      stopping=self.stopping,
    )

  def update(self, CC: structs.CarControl, CS: CarStateBase) -> None:
    """Update longitudinal control calculations.

    This is the main entry point called externally.

    Args:
        CC: Car control signals including actuators
        CS: Car state information
    """

    actuators = CC.actuators
    long_control_state = actuators.longControlState
    self.accel_cmd = CC.actuators.accel

    self.get_stopping_state(actuators)
    self.calculate_jerk(CC, CS, long_control_state)
    self.calculate_accel(CC)
    self.calculate_comfort_band(CC)
    self.get_tuning_state()

    self.long_control_state_last = long_control_state
