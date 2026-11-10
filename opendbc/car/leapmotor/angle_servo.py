import math
from collections import deque
from enum import IntEnum

import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.leapmotor.values import STEER_1_DT, CarControllerParams
from opendbc.car.vehicle_model import VehicleModel

CENTER_V, CENTER_ANGLE = (7.5, 16., 25., 37.5, 52.5, 77.5), (-2., -3., -3., -3.5, -3.5, -4.75)
CENTERING_V = (7.5, 9., 11., 13., 16., 19.5, 25., 37.5, 52.5, 77.5)
CENTERING_ANGLE = (0., 2., 5., 10., 20., 40., 80., 150., 300., 450., 560.)
CENTERING_TORQUE = (
  (0., 22., 58., 143., 175., 240., 306., 370., 440., 520., 600.),
  (0., 21., 62., 142., 180., 245., 300., 390., 480., 560., 630.),
  (0., 20., 66., 141., 187., 250., 300., 440., 560., 610., 650.),
  (0., 20., 70., 140., 193., 255., 300., 450., 560., 610., 650.),
  (0., 19., 76., 137., 204., 265., 300., 500., 610., 650., 650.),
  (0., 24., 83., 140., 182., 230., 340., 600., 650., 650., 650.),
  (0., 29., 90., 142., 165., 200., 387., 620., 650., 650., 650.),
  (0., 29., 73., 143., 177., 240., 330., 600., 650., 650., 650.),
  (0., 25., 77., 144., 216., 330., 520., 650., 650., 650., 650.),
  (0., 26., 66., 105., 200., 380., 650., 650., 650., 650., 650.),
)
DAMPING_V, DAMPING = (7.5, 16., 25., 37.5, 52.5), (0.81, 0.78, 0.47, 0.22, 0.2)

REF_HZ_BP, REF_HZ_V = (10., 40., 100.), (1.86, 1.49, 1.02)
REF_ZETA = 1.17
REF_JERK_BP, REF_JERK_V = (45., 60.), (3.0, 1.96)
RELEASE_JERK = 1.0
RELEASE_TIME = 1.5
RELEASE_RATE = 20.
DRIVER_OPPOSE = 60.
HANDS_TORQUE = 40.
REF_RATE_BP, REF_RATE_V = (5., 20., 40.), (450., 300., 60.)
REF_ACCEL = 1025.
LEAD = 0.042

# FREE
FRICTION = 62.
FRICTION_RATE = 5.9
INERTIA = 0.12
KP_BP, KP_V = (10., 40., 100.), (4., 3., 2.77)
KP_MAX = 79.
KD = 1.12
KD_MAX = 234.
KI_V, KI = (10., 30., 50.), (60., 30., 0.)
I_MAX_V, I_MAX = (10., 30., 50.), (250., 150., 0.)
I_TAU = 2.

# stuck
STILL_FRAMES = 3
MOVING_RATE = 8.
MOVED_ANGLE = 0.15
FREE_RATE_BP, FREE_RATE_V = (20., 60., 100.), (6.8, 6.6, 25.7)
DEADBAND_AY = 0.047
DEADBAND_MIN, DEADBAND_MAX = 0.67, 2.5
HIGH_SPEED = 45.
DEADBAND_MIN_V, DEADBAND_MIN_HIGH = (45., 60.), (DEADBAND_MIN, 1.0)
POST_BIG_HIGH = 2.5
BREAKAWAY_INIT = 100.
BREAKAWAY_LEARN = 0.25
BREAKAWAY_MIN, BREAKAWAY_MAX = 55., 150.
BIAS_MAX = 40.
BREAK_MARGIN = 5.
BREAK_RAMP = 1.4
BREAK_RAMP_ERROR = 1.68
BREAK_RAMP_ERROR_0 = 1.59
BREAK_RAMP_RATE = 0.018
DWELL_TIME = 0.4
DWELL_DROP = 4.
HOLD_MARGIN = 8.6
POST_FRAMES = 4
POST_TORQUE = -24.
POST_BIG = 1.09
SATURATED_AY = 0.6
FRICTION_TAU = 0.3
FRICTION_ERROR = 2.
REVERSE_TIME = 0.59
REVERSE_ERROR = 2.3


class Mode(IntEnum):
  FREE = 0
  HOLD = 1
  BREAK = 2
  POST = 3


def interp(x: float, xp, fp) -> float:
  return float(np.interp(x, xp, fp))


def clip(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else (hi if x > hi else x)


class AngleServo:
  def __init__(self, CP: structs.CarParams, params: CarControllerParams):
    self.params = params
    self.steer_max = params.STEER_MAX
    self.dt = DT_CTRL * params.STEER_STEP
    self.VM = VehicleModel(CP)

    self.breakaway: dict[int, float] = {1: BREAKAWAY_INIT, -1: BREAKAWAY_INIT}
    self.t = 0.
    self.last_break = {1: -9., -1: -9.}
    self.raw: deque[float] = deque(maxlen=3)
    self.angles: deque[float] = deque(maxlen=4)
    self.filt_last: float | None = None
    self.still = 0
    self.push_dir = 0
    self.reset(0.)

  def reset(self, angle: float):
    self.target = angle
    self.target_rate = 0.
    self.target_rate_lp = 0.
    self.target_accel = 0.
    self.mode = Mode.FREE
    self.dir = 0
    self.post = 0
    self.break_from = angle
    self.release_time = 0.
    self.saturated = False
    self._sat_error = 0.
    self._no_rise: tuple[int, float] | None = None
    self.last_break = {1: -9., -1: -9.}
    self.integral = 0.

  def observe(self, angle: float):
    if not math.isfinite(angle):
      return
    self.raw.append(angle)
    filt = sorted(self.raw)[len(self.raw) // 2]
    if self.filt_last is None:
      self.filt_last = filt
    self.still = 0 if abs(filt - self.filt_last) > 0.05 else self.still + 1
    self.filt_last = filt
    self.angles.append(filt)

  @property
  def rate(self) -> float:
    a = self.angles
    return (a[-1] - a[0]) / ((len(a) - 1) * STEER_1_DT) if len(a) >= 2 else 0.

  @property
  def bias(self) -> float:
    return clip(0.5 * (self.breakaway[1] - self.breakaway[-1]), -BIAS_MAX, BIAS_MAX)

  def ay_per_deg(self, v_ego: float) -> float:
    v = max(v_ego, 0.1)
    return v * v * self.VM.curvature_factor(v) * math.radians(1.) / self.VM.sR

  @staticmethod
  def centering(angle: float, v_kph: float) -> float:
    a = angle - interp(v_kph, CENTER_V, CENTER_ANGLE)
    return math.copysign(interp(v_kph, CENTERING_V, [np.interp(abs(a), CENTERING_ANGLE, row) for row in CENTERING_TORQUE]), a)

  def _update_target(self, v_ego: float, v_kph: float, angle_des: float) -> float:
    jerk = interp(v_kph, REF_JERK_BP, REF_JERK_V) if self.release_time >= RELEASE_TIME else RELEASE_JERK
    rate_max = min(jerk / max(self.ay_per_deg(v_ego), 1e-4), interp(v_kph, REF_RATE_BP, REF_RATE_V))
    wn = 2. * math.pi * interp(v_kph, REF_HZ_BP, REF_HZ_V)
    accel = wn * wn * (angle_des - self.target) - 2. * REF_ZETA * wn * self.target_rate
    rate = clip(self.target_rate + clip(accel, -REF_ACCEL, REF_ACCEL) * self.dt, -rate_max, rate_max)
    if self.release_time < RELEASE_TIME and rate * self.push_dir < 0.:
      rate = clip(rate, -RELEASE_RATE, RELEASE_RATE)
    self.target_accel = (rate - self.target_rate) / self.dt
    self.target_rate = rate
    self.target += rate * self.dt
    return self.target

  def update(self, v_ego: float, angle: float, angle_des: float, steer_rate: float, steering_pressed: bool,
             eps_active: bool, torque_last: int, driver_torque: float = 0.) -> float:
    """Every EPS_LKA_CMD frame it may steer: the torque to request, before apply_driver_steer_torque_limits."""
    v_kph = v_ego * 3.6
    if not (math.isfinite(v_ego) and math.isfinite(angle) and math.isfinite(angle_des)):
      self.reset(angle if math.isfinite(angle) else self.target)
      return 0.
    if steering_pressed or not eps_active:
      self.reset(angle)
      return 0.
    self.t += self.dt
    self.release_time += self.dt
    target = self._update_target(v_ego, v_kph, angle_des)
    error = target + LEAD * self.target_rate - angle
    rate = self.rate
    moving = self.still < STILL_FRAMES or abs(steer_rate) >= MOVING_RATE
    bias = self.bias
    neutral = self.centering(angle, v_kph)
    deadband = clip(DEADBAND_AY / max(self.ay_per_deg(v_ego), 1e-4), interp(v_kph, DEADBAND_MIN_V, DEADBAND_MIN_HIGH),
                    DEADBAND_MAX)
    s_err = 1 if error > 0. else -1
    high = v_kph >= HIGH_SPEED
    opposed = s_err * driver_torque <= -DRIVER_OPPOSE
    hands = abs(driver_torque) >= HANDS_TORQUE
    self._no_rise = (s_err, torque_last) if opposed else None

    friction = clip(self.target_rate / FRICTION_RATE, -1., 1.)
    self.target_rate_lp += (self.target_rate - self.target_rate_lp) * (1. - math.exp(-self.dt / FRICTION_TAU))
    low = interp(v_kph, (30., 40.), (1., 0.))
    if low > 0.:
      friction_low = s_err if abs(error) > FRICTION_ERROR * deadband else clip(self.target_rate_lp / FRICTION_RATE, -1., 1.)
      friction += low * (friction_low - friction)

    torque = self.centering(target + LEAD * self.target_rate, v_kph) + bias
    torque += FRICTION * friction
    torque += INERTIA * self.target_accel
    torque += interp(v_kph, DAMPING_V, DAMPING) * self.target_rate
    torque += clip(interp(v_kph, KP_BP, KP_V) * error, -KP_MAX, KP_MAX)
    torque += clip(KD * (self.target_rate - rate), -KD_MAX, KD_MAX)
    if abs(error) <= deadband:
      self.integral *= math.exp(-self.dt / I_TAU)
    elif opposed or hands:
      pass
    elif s_err * self.integral >= 0. and s_err * (torque + self.integral - torque_last) > \
         (self.params.STEER_DELTA_UP if s_err * torque_last >= 0. else self.params.STEER_DELTA_DOWN):
      pass
    elif not (abs(torque + self.integral) >= self.steer_max and (torque + self.integral) * error > 0.):
      i_max = interp(v_kph, I_MAX_V, I_MAX)
      self.integral = clip(self.integral + interp(v_kph, KI_V, KI) * error * self.dt, -i_max, i_max)
    torque += self.integral

    limited = abs(torque) >= self.steer_max and torque * error > 0.
    off = angle_des - angle
    self._sat_error = error if abs(error) > max(2.5, deadband) else 0.

    self.saturated = (limited and abs(error) > max(2.5, deadband)) or \
                     (self.release_time >= RELEASE_TIME and abs(off) > 2.5 and abs(off) * self.ay_per_deg(v_ego) > SATURATED_AY)

    if self.mode == Mode.BREAK:

      s = self.dir
      moved = s * (angle - self.break_from)
      moving = moved >= MOVED_ANGLE or (s * steer_rate >= 4. and moved >= 0.05)
    if moving:
      if self.mode == Mode.BREAK:
        s = self.dir
        self.last_break[s] = self.t
        if not hands:
          self.breakaway[s] = clip(self.breakaway[s] + BREAKAWAY_LEARN * (s * (torque_last - neutral) - self.breakaway[s]),
                                   BREAKAWAY_MIN, BREAKAWAY_MAX)
        runs_away = s * self.target_rate >= interp(v_kph, FREE_RATE_BP, FREE_RATE_V)
        if (abs(error) < (POST_BIG_HIGH if high else POST_BIG) * deadband) and not runs_away:
          self.mode, self.post = Mode.POST, POST_FRAMES
        else:
          self.mode = Mode.FREE
      if self.mode == Mode.POST:

        if self.post > 0 and s_err * self.dir > 0:
          self.post -= 1
          cap = neutral + bias + self.dir * POST_TORQUE
          return self._finish(min(torque, cap) if self.dir > 0 else max(torque, cap))
      self.mode = Mode.FREE
      return self._finish(torque)

    runs_away = s_err * self.target_rate >= interp(v_kph, FREE_RATE_BP, FREE_RATE_V)
    need = deadband * (REVERSE_ERROR if self.t - self.last_break[-s_err] < REVERSE_TIME else 1.)
    want = (abs(error) >= need and s_err * (error + 0.2 * self.target_rate) >= 0.5 * need) or \
           (runs_away and abs(error) >= 0.5 * need)
    if not want:
      self.mode = Mode.HOLD
      return self._finish(clip(torque, neutral - self.breakaway[-1] + HOLD_MARGIN, neutral + self.breakaway[1] - HOLD_MARGIN))

    s = s_err
    if self.mode != Mode.BREAK or self.dir != s:
      self.break_from = angle
    self.mode, self.dir = Mode.BREAK, s
    fresh = max(1. - self.still * STEER_1_DT / DWELL_TIME, 0.)
    edge = neutral + s * (self.breakaway[s] - BREAK_MARGIN - DWELL_DROP * fresh)
    up = self.params.STEER_DELTA_UP
    if s * (torque_last - edge) < -1.:
      nxt: float = torque_last + s * up
      nxt = min(nxt, edge) if s > 0 else max(nxt, edge)
    else:
      ramp = BREAK_RAMP + BREAK_RAMP_ERROR * max(abs(error) / deadband - BREAK_RAMP_ERROR_0, 0.) + \
             BREAK_RAMP_RATE * abs(self.target_rate)
      nxt = torque_last + s * min(ramp, up)

    if s * (torque - nxt) > 0 and abs(error) >= POST_BIG * deadband:
      nxt = torque
    return self._finish(nxt)

  def _finish(self, torque: float) -> float:
    torque = clip(torque, -self.steer_max, self.steer_max)
    if self._no_rise is not None:
      s, last = self._no_rise
      if s * (torque - last) > 0.:
        torque = last

    if abs(torque) >= self.steer_max - 0.5 and torque * self._sat_error > 0.:
      self.saturated = True
    return torque
