import time


class TorqueReductionGainController:
  def __init__(self, angle_threshold=3.0, debounce_time=0.5, min_gain=0.0, max_gain=1.0, ramp_up_rate=0.1,
               ramp_down_rate=0.05):
    """
    angle_threshold: degrees difference to consider as 'saturated'
    debounce_time: seconds to wait before increasing gain
    min_gain: minimum torque reduction gain (0 = no torque)
    max_gain: maximum torque reduction gain (1 = max torque)
    ramp_up_rate: gain increase per second when saturated
    ramp_down_rate: gain decrease per second when not saturated
    """
    self.angle_threshold = angle_threshold
    self.debounce_time = debounce_time
    self.min_gain = min_gain
    self.max_gain = max_gain
    self.ramp_up_rate = ramp_up_rate
    self.ramp_down_rate = ramp_down_rate
    self.saturated_since = None
    self.gain = min_gain
    self.last_update_time = time.monotonic()

  def update(self, params, last_requested_angle, actual_angle, lat_active):
    self.min_gain = params.ANGLE_ACTIVE_TORQUE_REDUCTION_GAIN
    self.max_gain = params.ANGLE_MAX_TORQUE_REDUCTION_GAIN
    self.ramp_up_rate = params.ANGLE_RAMP_UP_TORQUE_REDUCTION_RATE
    self.ramp_down_rate = params.ANGLE_RAMP_DOWN_TORQUE_REDUCTION_RATE

    now = time.monotonic()
    dt = now - self.last_update_time
    self.last_update_time = now

    angle_error = abs(last_requested_angle - actual_angle)
    saturated = lat_active and angle_error > self.angle_threshold

    if saturated:
      if self.saturated_since is None:
        self.saturated_since = now
      elif (now - self.saturated_since) > self.debounce_time:
        self.gain = min(self.gain + self.ramp_up_rate * dt, self.max_gain)
    else:
      self.saturated_since = None
      self.gain = max(self.gain - self.ramp_down_rate * dt, self.min_gain)

    if not lat_active:
      self.gain = self.min_gain
      self.saturated_since = None

    return self.gain

  def reset(self):
    self.gain = self.min_gain
    self.saturated_since = None
    self.last_update_time = time.monotonic()
