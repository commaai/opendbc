import bisect
from numbers import Number


def _interp(x: float, xp: list, fp: list) -> float:
  if x <= xp[0]:
    return float(fp[0])
  if x >= xp[-1]:
    return float(fp[-1])
  i = bisect.bisect_right(xp, x) - 1
  t = (x - xp[i]) / (xp[i + 1] - xp[i])
  return float(fp[i] + t * (fp[i + 1] - fp[i]))


class PIDController:
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.speed = 0.0

    self.reset()

  @property
  def k_p(self):
    return _interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return _interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return _interp(self.speed, self._k_d[0], self._k_d[1])

  @property
  def error_integral(self):
    return self.i/self.k_i

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if override:
      self.i -= self.i_unwind_rate * (1.0 if self.i > 0 else (-1.0 if self.i < 0 else 0.0))
    else:
      if not freeze_integrator:
        self.i = self.i + error * self.k_i * self.i_rate

        # Clip i to prevent exceeding control limits
        control_no_i = self.p + self.d + self.f
        control_no_i = max(self.neg_limit, min(self.pos_limit, control_no_i))
        self.i = max(self.neg_limit - control_no_i, min(self.pos_limit - control_no_i, self.i))

    control = self.p + self.i + self.d + self.f

    self.control = max(self.neg_limit, min(self.pos_limit, control))
    return self.control
