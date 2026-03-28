import numpy as np
from opendbc.car.common.conversions import Conversions as CV


class TorqueReductionGainController:
  """
  Controls the ADAS_ACIAnglTqRedcGainVal signal for HKG CAN-FD angle steering.

  The gain controls how hard the EPS tries to track the commanded angle.
  The angle command is always sent directly from the model — the gain only
  controls the EPS effort to reach it.

  EPS whine at low speed is caused by the EPS internal PID resonating when
  gain is too high. Route 16 data showed oscillation starts above gain ~0.35
  at speeds below 30 km/h. However, an error-based approach to limit gain
  creates a deadlock: low gain → EPS can't close error → error stays high →
  gain stays low. The ceiling approach avoids this by limiting the steady-state
  gain regardless of error — the EPS works within its stable range.
  """

  # Speed-dependent gain ceiling. Tuned to stay below the EPS oscillation
  # threshold at each speed while maintaining enough assist to steer.
  # Route 16: gain < 0.3 = quiet, gain > 0.5 = whine at low speed.
  # Route 17: gain 0.13-0.20 = no steering at all (too low).
  # Sweet spot: 0.35 at low speed — below oscillation, above minimum for steering.
  SPEED_BP = [0., 10., 30., 50., 80.]  # km/h
  SPEED_CEILING = [0.35, 0.35, 0.55, 0.80, 1.0]

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
      target = ceiling * self.OVERRIDE_FACTOR if steering_pressed else ceiling

    if steering_pressed:
      self._was_overriding = True
    elif self._was_overriding and lat_active and self.gain >= target - 0.001:
      self._was_overriding = False

    # Ramp toward target
    if target < self.gain:
      if steering_pressed:
        rate = self.OVERRIDE_DROP_RATE
      elif not lat_active:
        rate = self.RAMP_RATE
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
