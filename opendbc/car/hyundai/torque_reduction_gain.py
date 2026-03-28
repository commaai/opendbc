import numpy as np


class TorqueReductionGainController:
  """
  Controls the ADAS_ACIAnglTqRedcGainVal signal for HKG CAN-FD angle steering.

  This is a normalized multiplier (0.0–1.0) that controls how much electrical
  assist the MDPS applies when tracking the commanded steering angle.

  Stock ADAS behavior (observed from route data):
    - When active and no driver override: gain ramps up to ~0.9–1.0
    - When driver overrides: gain reduces proportionally to driver torque,
      but never reaches zero — maintains ~0.08 minimum assist
    - When inactive: gain is 0
    - Ramp up: ~0.012/frame at 100Hz
    - Ramp down: ~0.008/frame at 100Hz during override, ~0.020/frame on deactivation
  """

  def __init__(self):
    self.gain = 0.0

  def update(self, driver_torque: float, steering_pressed: bool, lat_active: bool,
             steer_threshold: float) -> float:
    if not lat_active:
      # Ramp down to zero when inactive (~0.020/frame, reaches 0 from 1.0 in ~0.5s)
      self.gain = max(self.gain - 0.020, 0.0)
      return self.gain

    if steering_pressed:
      # Driver is overriding: target gain based on torque magnitude
      # Stock data shows roughly linear relationship:
      #   |torque| ~threshold -> gain ~0.85
      #   |torque| ~2x threshold -> gain ~0.58
      #   |torque| ~3x threshold -> gain ~0.25
      #   |torque| ~4x+ threshold -> gain ~0.08
      # Approximated as: clip(1.0 - |torque| / (threshold * 4.5), 0.08, 1.0)
      target = float(np.clip(1.0 - abs(driver_torque) / (steer_threshold * 4.5), 0.08, 1.0))

      # Ramp toward target (~0.008/frame when reducing)
      if target < self.gain:
        self.gain = max(self.gain - 0.008, target)
      else:
        self.gain = min(self.gain + 0.012, target)
    else:
      # No override: ramp up toward 1.0 (~0.012/frame, reaches 1.0 from 0 in ~0.8s)
      self.gain = min(self.gain + 0.012, 1.0)

    return self.gain
