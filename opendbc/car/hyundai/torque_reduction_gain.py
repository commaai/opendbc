import numpy as np
from opendbc.car.common.conversions import Conversions as CV


class TorqueReductionGainController:
  """
  Controls the ADAS_ACIAnglTqRedcGainVal signal for HKG CAN-FD angle steering.

  The gain controls how hard the EPS tries to track the commanded angle:
    - Speed-dependent ceiling: higher at highway speed for precision,
      lower at low speed to reduce EPS internal PID oscillation.
    - Override: drops gain when steeringPressed for easy driver takeover.
    - Smooth ramp: prevents sudden gain changes that jerk the steering.
  """

  SPEED_BP = [0., 10., 30., 50., 80.]  # km/h
  SPEED_CEILING = [0.55, 0.55, 0.75, 0.90, 1.0]

  OVERRIDE_FACTOR = 0.1

  RAMP_RATE = 0.008
  RECOVERY_RATE = 0.02
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
      self._was_overriding = False

    if target < self.gain:
      rate = self.OVERRIDE_DROP_RATE if steering_pressed else self.RAMP_RATE
      self.gain = max(self.gain - rate, target)
    else:
      rate = self.RECOVERY_RATE if self._was_overriding else self.RAMP_RATE
      self.gain = min(self.gain + rate, target)

    return self.gain
