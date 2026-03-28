import numpy as np
from opendbc.car.common.conversions import Conversions as CV


class TorqueReductionGainController:
  """
  Controls the ADAS_ACIAnglTqRedcGainVal signal for HKG CAN-FD angle steering.

  The gain controls how hard the EPS tries to track the commanded angle.
  Instead of smoothing the angle command (which delays the model's intent),
  we modulate the gain to control HOW AGGRESSIVELY the EPS follows it.

  When the angle command is changing rapidly (large |dangle|), the gain is
  reduced so the EPS eases into the new angle instead of slamming. When
  commands are stable, gain is high for precise tracking.

  No feedback loop risk: gain reacts to command rate |dangle|, which is
  independent of the gain itself.
  """

  # Speed-dependent gain ceiling. At low speed, EPS internal PID oscillates
  # (overshoots, reverses, resonates) when gain is too high relative to the
  # angle error — this is the audible EPS whine on micro-adjustments.
  # Lower ceiling at low speed prevents the oscillation.
  SPEED_BP = [0.,  10.,  30.,  50.,  80.]  # km/h
  SPEED_CEILING = [0.55, 0.55, 0.75, 0.90, 1.0]

  # When steeringPressed: drop gain to this fraction of ceiling.
  # Must be low enough that the driver doesn't have to fight the EPS.
  # At 0.1 × 0.85 = 0.17 (low speed) or 0.2 × 1.0 = 0.2 (highway),
  # EPS only applies ~15-20% effort — easy to override.
  OVERRIDE_FACTOR = 0.1

  # When angle command changes rapidly, reduce gain target.
  # VM rate-limits dangle to ~0.3-0.5°/frame at most speeds — normal steering.
  # Only large steps (>0.5°, which are rare after VM) need gain reduction.
  DANGLE_BP = [0., 0.5, 1.0, 2.0, 5.0]  # degrees per frame (post VM limiting)
  DANGLE_FACTOR = [1.0, 1.0, 0.7, 0.5, 0.3]

  RAMP_RATE = 0.008  # normal ramp up/down
  RECOVERY_RATE = 0.02  # faster recovery after override
  OVERRIDE_DROP_RATE = 0.05  # fast drop during override

  def __init__(self):
    self.gain = 0.0
    self.last_angle = 0.0
    self._was_overriding = False

  def update(self, steering_pressed: bool, lat_active: bool, v_ego: float, apply_angle: float) -> float:
    dangle = abs(apply_angle - self.last_angle)
    # Winding = |angle| increasing (EPS building torque into a turn, slap risk)
    # Unwinding = |angle| decreasing (EPS releasing torque, no slap risk)
    winding = abs(apply_angle) > abs(self.last_angle)
    self.last_angle = apply_angle

    if not lat_active:
      target = 0.0
      self._was_overriding = False
    else:
      speed_kmh = v_ego * CV.MS_TO_KPH
      ceiling = float(np.interp(speed_kmh, self.SPEED_BP, self.SPEED_CEILING))

      if steering_pressed:
        target = ceiling * self.OVERRIDE_FACTOR
      elif winding:
        # Only reduce gain when winding (|angle| increasing = EPS building torque).
        # Unwinding gets full gain so the wheel returns to center quickly.
        dangle_factor = float(np.interp(dangle, self.DANGLE_BP, self.DANGLE_FACTOR))
        target = ceiling * dangle_factor
      else:
        target = ceiling

    if steering_pressed:
      self._was_overriding = True
    elif self._was_overriding and lat_active and self.gain >= target - 0.001:
      self._was_overriding = False

    # Ramp toward target
    if target < self.gain:
      if steering_pressed:
        rate = self.OVERRIDE_DROP_RATE
      else:
        rate = self.RAMP_RATE
      self.gain = max(self.gain - rate, target)
    else:
      if self._was_overriding:
        rate = self.RECOVERY_RATE
      else:
        rate = self.RAMP_RATE
      self.gain = min(self.gain + rate, target)

    return self.gain
