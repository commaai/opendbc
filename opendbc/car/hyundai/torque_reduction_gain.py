import numpy as np
from opendbc.car.common.conversions import Conversions as CV


class TorqueReductionGainController:
  """
  Controls the ADAS_ACIAnglTqRedcGainVal signal for HKG CAN-FD angle steering.

  This is a normalized multiplier (0.0-1.0) that controls how much electrical
  assist the MDPS applies when tracking the commanded steering angle.

  Stock ADAS behavior (from routes 2 and 8):
    - At 10-50 km/h: gain ceiling ~0.85, even at high steering angles (50-70°)
    - At 50-80 km/h: ceiling ~0.96
    - At 80+ km/h: ceiling reaches 1.0
    - The very low gain seen at 0-2 km/h (~0.04) is from the ramp-up after
      activation, NOT from a speed-based ceiling — stock uses 0.85 once ramped
    - Driver torque reduces gain continuously and proportionally
    - Post-override recovery ramps at ~0.004/frame
    - Below 0.8, the EPS cannot steer effectively at high angles (user tested)
  """

  # Speed-dependent gain ceiling. Stock data shows the ceiling steps up
  # with speed. Below 50 km/h the max observed is ~0.85, which is enough
  # for the EPS to track any angle the car can request.
  SPEED_BP =       [0.,  10.,  50.,  80.]  # km/h
  SPEED_CEILING =  [0.85, 0.85, 0.96, 1.0]

  # Driver torque override factor. Reduces gain proportionally to driver
  # torque so override is comfortable. At 250 Nm, gain drops to ~55% of
  # ceiling. At 750+ Nm, near-zero assist.
  # Derived from stock route 8 (gain / ceiling at various torque levels).
  TORQUE_BP =      [0.,  100., 200., 300., 400., 500., 750.]
  TORQUE_FACTOR =  [1.0, 0.85, 0.57, 0.48, 0.40, 0.33, 0.10]

  RAMP_UP_RATE = 0.05    # max gain increase per frame (~5.0/s at 100Hz, stock ~0.004)
  RAMP_DOWN_RATE = 0.08  # max gain decrease per frame (~8.0/s, faster for responsive override)

  def __init__(self):
    self.gain = 0.0

  def update(self, driver_torque: float, lat_active: bool, v_ego: float) -> float:
    if not lat_active:
      target = 0.0
    else:
      speed_kmh = v_ego * CV.MS_TO_KPH
      ceiling = float(np.interp(speed_kmh, self.SPEED_BP, self.SPEED_CEILING))
      override = float(np.interp(abs(driver_torque), self.TORQUE_BP, self.TORQUE_FACTOR))
      target = ceiling * override

    # Smooth ramp toward target (down faster for responsive override)
    if target < self.gain:
      self.gain = max(self.gain - self.RAMP_DOWN_RATE, target)
    else:
      self.gain = min(self.gain + self.RAMP_UP_RATE, target)

    return self.gain
