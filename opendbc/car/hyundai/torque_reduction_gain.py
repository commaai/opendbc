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

  SPEED_BP = [0.,  10.,  50.,  80.]  # km/h
  SPEED_CEILING = [0.85, 0.85, 0.96, 1.0]

  # When steeringPressed: drop gain to this fraction of ceiling.
  # Must be low enough that the driver doesn't have to fight the EPS.
  # At 0.1 × 0.85 = 0.17 (low speed) or 0.2 × 1.0 = 0.2 (highway),
  # EPS only applies ~15-20% effort — easy to override.
  OVERRIDE_FACTOR = 0.2

  # When angle command changes rapidly, reduce gain instantly.
  # Small corrections (< 0.3°) don't reduce gain — only large steps matter.
  DANGLE_BP = [0., 0.3, 0.5, 1.0, 2.0]  # degrees per frame (post VM limiting)
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
    self.last_angle = apply_angle

    if not lat_active:
      target = 0.0
      self._was_overriding = False
    else:
      speed_kmh = v_ego * CV.MS_TO_KPH
      ceiling = float(np.interp(speed_kmh, self.SPEED_BP, self.SPEED_CEILING))

      if steering_pressed:
        target = ceiling * self.OVERRIDE_FACTOR
      else:
        dangle_factor = float(np.interp(dangle, self.DANGLE_BP, self.DANGLE_FACTOR))
        target = ceiling * dangle_factor

    if steering_pressed:
      self._was_overriding = True
    elif self._was_overriding and lat_active and self.gain >= target - 0.001:
      self._was_overriding = False

    # Apply dangle reduction instantly (no ramp — catches the frame the command jumps)
    if lat_active and not steering_pressed:
      dangle_factor = float(np.interp(dangle, self.DANGLE_BP, self.DANGLE_FACTOR))
      instant_limit = self.gain * dangle_factor
      if instant_limit < self.gain:
        self.gain = instant_limit

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
