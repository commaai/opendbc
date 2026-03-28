import numpy as np
from opendbc.car.common.conversions import Conversions as CV


class TorqueReductionGainController:
  """
  Controls the ADAS_ACIAnglTqRedcGainVal signal for HKG CAN-FD angle steering.

  This is a normalized multiplier (0.0-1.0) that controls how much electrical
  assist the MDPS applies when tracking the commanded steering angle.

  Ramp rates are split by context:
    - Activation/deactivation: slow ramp to prevent EPS motor noise. The EPS
      adjusts its output proportionally to gain × angle_error each frame.
      Fast gain changes cause step changes in EPS torque = audible whine.
      Stock ramps at ~0.004/frame. We use 0.008 (2x stock) for faster engage.
    - Override drop: can be faster since the driver is actively steering and
      their input masks any EPS motor noise.
    - Override recovery: slow to prevent sudden snap-back when the system
      takes back control ("safe re-engage").
  """

  SPEED_BP = [0.,  10.,  50.,  80.]  # km/h
  SPEED_CEILING = [0.85, 0.85, 0.96, 1.0]

  # When steeringPressed: drop gain to this fraction of ceiling.
  OVERRIDE_FACTOR = 0.6

  # Activation/deactivation: slow to avoid EPS whine (stock ~0.004)
  RAMP_RATE = 0.008

  # Recovery after override: faster than activation so the system can snap
  # back to lane if the driver releases mid-lane-change, but not instant.
  RECOVERY_RATE = 0.02

  # Override: fast drop since driver input masks EPS noise
  OVERRIDE_DROP_RATE = 0.05

  def __init__(self):
    self.gain = 0.0
    self._was_overriding = False

  def update(self, steering_pressed: bool, lat_active: bool, v_ego: float) -> float:
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
      # Clear override flag only once gain has fully recovered to ceiling
      self._was_overriding = False

    # Pick ramp rate based on context
    if target < self.gain:
      if steering_pressed:
        rate = self.OVERRIDE_DROP_RATE  # fast drop — driver masks EPS noise
      else:
        rate = self.RAMP_RATE  # slow deactivation — no whine
      self.gain = max(self.gain - rate, target)
    else:
      if self._was_overriding:
        # Recovering from override — faster rate so system can snap
        # back to lane if driver releases mid-maneuver
        rate = self.RECOVERY_RATE
      else:
        rate = self.RAMP_RATE  # normal activation — slow, no whine
      self.gain = min(self.gain + rate, target)

    return self.gain
