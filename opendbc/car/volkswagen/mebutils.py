import numpy as np

from opendbc.car.common.pid import PIDController


class MultiplicativeUnwindPID(PIDController):
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100, min_cmd=1e-10, ki_red_time=1.0):
    super().__init__(k_p, k_i, k_f=k_f, k_d=k_d, pos_limit=pos_limit, neg_limit=neg_limit, rate=rate)
    self.min_cmd = abs(min_cmd)
    self.ki_red_time = float(ki_red_time)
    self.rate = rate
    self.override_prev = False
    self.i_unwind_factor = 1.0

  def _calc_unwind_factor(self, override):
    if not override or self.override_prev:
      return
    if self.ki_red_time <= 0.0:
      self.i_unwind_factor = 1.0
      return
    if abs(self.i) <= self.min_cmd:
      self.i_unwind_factor = 0.0
      return
    steps = max(int(self.ki_red_time * self.rate), 1)
    factor = (self.min_cmd / abs(self.i)) ** (1.0 / steps)
    self.i_unwind_factor = min(factor, 1.0)
    
  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if override:
      self._calc_unwind_factor(override)
      self.i *= self.i_unwind_factor
      if abs(self.i) < self.min_cmd:
        self.i = 0.0
    else:
      if not freeze_integrator:
        self.i = self.i + error * self.k_i * self.i_rate

        # Clip i to prevent exceeding control limits
        control_no_i = self.p + self.d + self.f
        control_no_i = np.clip(control_no_i, self.neg_limit, self.pos_limit)
        self.i = np.clip(self.i, self.neg_limit - control_no_i, self.pos_limit - control_no_i)

    control = self.p + self.i + self.d + self.f

    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    self.override_prev = override
    return self.control

class LatControlCurvature():
  def __init__(self, pid_params, limit, rate):
    self.pid = MultiplicativeUnwindPID((pid_params.kpBP, pid_params.kpV),
                                       (pid_params.kiBP, pid_params.kiV),
                                       k_f=pid_params.kf, pos_limit=limit, neg_limit=-limit,
                                       rate=rate, min_cmd=6.7e-6, ki_red_time=2.0)
  def reset(self):
    self.pid.reset()
  
  def update(self, CS, CC, desired_curvature):
    actual_curvature_vm    = CC.currentCurvature # includes roll
    actual_curvature_pose  = CC.angularVelocity[2] / max(CS.vEgo, 0.1)
    actual_curvature       = np.interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_pose])
    desired_curvature_corr = desired_curvature - CC.rollCompensation
    error                  = desired_curvature - actual_curvature
    freeze_integrator      = CC.steerLimited or CS.vEgo < 5
    output_curvature       = self.pid.update(error, feedforward=desired_curvature_corr, speed=CS.vEgo,
                                             freeze_integrator=freeze_integrator, override=CS.steeringPressed)
    return output_curvature