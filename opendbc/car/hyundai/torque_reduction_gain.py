import numpy as np
from opendbc.car.common.conversions import Conversions as CV


class TorqueReductionGainController:
  """
  Controls the ADAS_ACIAnglTqRedcGainVal signal for HKG CAN-FD angle steering.

  This is a normalized multiplier (0.0-1.0) that controls how much electrical
  assist the MDPS applies when tracking the commanded steering angle.

  Override philosophy (from maintainer and stock analysis):
    - During normal lane-keeping, EPS output is only 2-3 Nm regardless of gain
      level, because angle error is small. This is already "1 finger" overridable.
    - When the driver overrides, the controls layer detects it via steeringPressed
      and adjusts the angle command toward the driver's steering. The angle error
      drops, and with it the EPS resistance — this is the primary override mechanism.
    - The gain reduction during override is secondary feedback: it provides tactile
      confirmation that the system is yielding, and ensures the EPS doesn't fight
      the driver during the few frames before controls adapts.
    - After override release, slow ramp-up ensures smooth "re-engagement" without
      a sudden jerk as the system takes back control.

  Stock behavior (from route analysis):
    - Gain barely changes during torque spikes (road noise, bumps). Stock ignores
      raw torque for gain control.
    - During sustained override, gain drops at most 0.008-0.020/frame.
    - Ramp rates: up ~0.004/frame (median), down ~0.008/frame (median).
    - Gain ceiling: ~0.85 at low speed, ~0.96 at highway, 1.0 at 100+ km/h.
  """

  SPEED_BP =       [0.,  10.,  50.,  80.]  # km/h
  SPEED_CEILING =  [0.85, 0.85, 0.96, 1.0]

  # When steeringPressed: drop gain to this fraction of ceiling.
  # Not aggressive — the controls layer handles the real disengagement.
  # This just provides tactile "yielding" feedback.
  OVERRIDE_FACTOR = 0.6

  RAMP_UP_RATE = 0.05    # ~5.0/s — close to stock median of 0.004
  RAMP_DOWN_RATE = 0.08  # ~8.0/s — slightly faster than stock for responsive feel

  def __init__(self):
    self.gain = 0.0

  def update(self, steering_pressed: bool, lat_active: bool, v_ego: float) -> float:
    if not lat_active:
      target = 0.0
    else:
      speed_kmh = v_ego * CV.MS_TO_KPH
      ceiling = float(np.interp(speed_kmh, self.SPEED_BP, self.SPEED_CEILING))
      target = ceiling * self.OVERRIDE_FACTOR if steering_pressed else ceiling

    # Smooth ramp toward target
    if target < self.gain:
      self.gain = max(self.gain - self.RAMP_DOWN_RATE, target)
    else:
      self.gain = min(self.gain + self.RAMP_UP_RATE, target)

    return self.gain
