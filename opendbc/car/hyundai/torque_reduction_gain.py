import numpy as np
from opendbc.car.common.conversions import Conversions as CV


class TorqueReductionGainController:
  """
  Controls the ADAS_ACIAnglTqRedcGainVal signal for HKG CAN-FD angle steering.

  The gain controls how hard the EPS tries to track the commanded angle.

  EPS whine happens when gain × angle_error is too high — the EPS internal PID
  oscillates (overshoots, reverses, resonates). Data from route 16 shows:
    - gain < 0.3 with any error: ~1-2% EPS oscillation (quiet)
    - gain 0.5-0.6 with large error: ~20% oscillation (loud whine)
    - gain 0.8-0.9 with small error: ~3% oscillation (quiet)

  The product (gain × error) is what matters, not gain alone. So we limit gain
  based on the current angle error: when error is large, gain stays low to prevent
  oscillation. As the EPS tracks and error shrinks, gain ramps up for precision.

  No feedback loop: error = (angle_cmd - angle_est). The angle command doesn't
  change based on gain, and angle_est is measured from the steering column.
  """

  SPEED_BP = [0., 10., 30., 50., 80.]  # km/h
  SPEED_CEILING = [0.85, 0.85, 0.90, 0.96, 1.0]

  # Gain limit based on angle error. Route 16 data shows EPS oscillation
  # even at 0.5° error when gain > 0.5 at low speed. The EPS internal PID
  # resonates when trying to track small errors at moderate-high gain.
  # Only truly zero error is safe at full gain.
  ANGLE_ERROR_BP = [0., 0.3, 0.8, 2., 5., 15.]  # degrees
  ERROR_FACTOR =   [1.0, 0.6, 0.35, 0.25, 0.18, 0.12]

  OVERRIDE_FACTOR = 0.1

  RAMP_RATE = 0.008
  RECOVERY_RATE = 0.02
  OVERRIDE_DROP_RATE = 0.05

  def __init__(self):
    self.gain = 0.0
    self._was_overriding = False

  def update(self, steering_pressed: bool, lat_active: bool, v_ego: float,
             apply_angle: float, steering_angle: float) -> float:
    if not lat_active:
      target = 0.0
      self._was_overriding = False
    else:
      speed_kmh = v_ego * CV.MS_TO_KPH
      ceiling = float(np.interp(speed_kmh, self.SPEED_BP, self.SPEED_CEILING))

      if steering_pressed:
        target = ceiling * self.OVERRIDE_FACTOR
      else:
        # Limit gain based on angle error to prevent EPS oscillation.
        # Large error + high gain = EPS PID resonance = audible whine.
        angle_error = abs(apply_angle - steering_angle)
        error_factor = float(np.interp(angle_error, self.ANGLE_ERROR_BP, self.ERROR_FACTOR))
        target = ceiling * error_factor

    if steering_pressed:
      self._was_overriding = True
    elif self._was_overriding and lat_active and self.gain >= target - 0.001:
      self._was_overriding = False

    # Error-based cap is applied instantly (no ramp) — the EPS must not
    # push hard when there's a gap to close, or it oscillates.
    # The ramp only applies for activation, deactivation, and override.
    if target < self.gain:
      if steering_pressed:
        self.gain = max(self.gain - self.OVERRIDE_DROP_RATE, target)
      elif not lat_active:
        self.gain = max(self.gain - self.RAMP_RATE, target)
      else:
        # Error-based reduction: apply instantly
        self.gain = target
    else:
      if self._was_overriding:
        rate = self.RECOVERY_RATE
      else:
        rate = self.RAMP_RATE
      self.gain = min(self.gain + rate, target)

    return self.gain
