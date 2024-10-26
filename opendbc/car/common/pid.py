from numbers import Number
from opendbc.car.common.numpy_fast import clip, interp


class PIDController:
  def __init__(self, k_p, k_i, pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = k_p
    self._k_i = k_i
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.speed = 0.0

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def error_integral(self):
    return self.i/self.k_i

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.control = 0

  def update(self, error, speed=0.0, freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p

    i = self.i + error * self.k_i * self.i_rate
    control = self.p + i

    # Update when changing i will move the control away from the limits
    # or when i will move towards the sign of the error
    if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
        (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
       not freeze_integrator:
      self.i = i

    control = self.p + self.i

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
