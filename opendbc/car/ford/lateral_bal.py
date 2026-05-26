"""
Ford CAN-FD lateral control — PSCM-inverse "bal" allocation.

Inputs:
  desired_curvature (1/m) from the planner — already contains turning intent
  plus drift correction.

Output: a path polynomial (c0, c1, c2, curvature_rate) for PSCM's
LateralMotionControl2 message such that the curvature PSCM delivers at the
wheels matches the requested desired_curvature as closely as possible.

Method:
  1. The PSCM polynomial-to-curvature transfer was characterized empirically
     across ~49,000 active driving frames; per-speed-bin coefficients
     α(v), β(v), γ(v) such that act_κ ≈ α·c0 + β·c1 + γ·c2.
  2. To deliver |desk|, pick a utilization u = |desk|/(β·L_c1 + γ·L_c2) and
     emit c1 = sign(desk)·u·L_c1, c2 = sign(desk)·u·L_c2. Channels with
     non-positive coefficient at that speed are skipped (they would actively
     fight desk — observed e.g. γ<0 below 3 m/s on the platforms measured).
  3. If c1+c2 alone are insufficient (u_c12 > 1), the c0 channel is brought
     in as overflow at the same utilization to extend authority. This keeps
     c0 silent in normal driving (where α is small and uncertain) and only
     uses it at the hardware ceiling.
  4. A per-platform scalar multiplies β and γ to absorb vehicle-to-vehicle
     calibration differences (mass/wheelbase/PSCM-cal trim). Default is 1.0
     for any platform without an explicit entry.
  5. An optional online estimator (FordBalLiveScale) observes (desk, act_κ)
     on stable curves and slowly adjusts a per-vehicle live multiplier
     bounded ±15% of the platform default. Persists via openpilot Params
     when available; runs in-memory otherwise.

Curvature rate (4th polynomial slot, LatCtlCrv_NoRate2_Actl) is the rate
of change of curvature over distance, dκ/dx = (Δdesk/Δt) / v_ego, clipped
to the DBC range ±0.001023 1/m². Computed by the carcontroller.
"""
from __future__ import annotations
from typing import Optional, Tuple


# ─── PSCM transfer function (regression table) ───────────────────────────────
# Empirical multi-variate regression over 49,117 active frames / 31 routes /
# 93 segments. Intercept dropped (it captured road-crown bias, ≤7e-3).
#
#  (v_max m/s,   α,         β,         γ,         R²)
FORD_PSCM_REGRESSION: Tuple[Tuple[float, float, float, float, float], ...] = (
  ( 3.0,  0.00903,  0.01708, -0.15194, 0.45),  # parking — degenerate, low R²
  ( 6.0,  0.00914,  0.06149,  0.39633, 0.78),
  (10.0,  0.00134,  0.08842,  0.61726, 0.83),
  (15.0,  0.00157,  0.06843,  0.64632, 0.87),
  (22.0,  0.00070,  0.05117,  0.79698, 0.93),  # highway — best fit
  (35.0, -0.00191,  0.04402,  0.63223, 0.89),
)

# Per-platform multiplier on β/γ to absorb per-vehicle calibration differences.
# scale > 1.0 → shrinks bal's u allocation → less c1+c2 sent (cancels over-delivery).
# scale < 1.0 → grows u → more c1+c2 sent (cancels under-delivery).
# Empty by default — all platforms start at 1.0 and the live estimator
# discovers each vehicle's actual offset from there.
FORD_PSCM_PLATFORM_SCALE: dict[str, float] = {}
FORD_PSCM_PLATFORM_SCALE_DEFAULT = 1.0

# Per-channel DBC clip magnitudes (LateralMotionControl2 signal ranges).
FORD_BAL_LC0 = 5.11    # m
FORD_BAL_LC1 = 0.5235  # rad
FORD_BAL_LC2 = 0.02    # 1/m

# ─── Live estimator constants ─────────────────────────────────────────────────
# Slow first-order filter on the live scalar, bounded ±25% from platform default.
# Robustness layers modeled on selfdrive/locationd/torqued.py — including a
# ramping filter decay that converges fast initially and slows as data builds,
# so single bad routes can't yank a well-converged value.
FORD_BAL_LIVE_VERSION         = "v1"   # bump to invalidate persisted values
FORD_BAL_LIVE_BOUND_LOW       = 0.75   # ±25% from platform default
FORD_BAL_LIVE_BOUND_HIGH      = 1.25
FORD_BAL_LIVE_MIN_DECAY       = 50.0   # initial filter decay (alpha ≈ 0.02 — fast)
FORD_BAL_LIVE_MAX_DECAY       = 500.0  # final filter decay   (alpha ≈ 0.002 — stable)
FORD_BAL_LIVE_DECAY_STEP      = 1.0    # decay increment per successful update
FORD_BAL_LIVE_MIN_FRAMES      = 300    # ~3 s @ 100 Hz before first update
FORD_BAL_LIVE_BUCKET_MIN_PT   = 20     # frames per bucket before it counts
FORD_BAL_LIVE_BUCKETS_V       = (3.0, 6.0, 10.0, 15.0, 22.0, 35.0)
FORD_BAL_LIVE_DESK_MIN        = 0.003  # |desk| floor for inclusion
FORD_BAL_LIVE_V_MIN           = 3.0    # m/s — exclude parking
FORD_BAL_LIVE_DESK_VAR_MAX    = 0.002  # std over 1 s window; reject volatile desk
FORD_BAL_LIVE_DESK_BUF        = 20     # ~1 s @ 20 Hz fresh data
FORD_BAL_LIVE_PERSIST_FRAMES  = 6000   # write to Params every ~60 s @ 100 Hz

# Soft import — opendbc as a library doesn't require openpilot. If Params
# isn't available the estimator runs in-memory and resets each drive.
try:
  from openpilot.common.params import Params  # type: ignore
  _HAS_PARAMS = True
except Exception:
  Params = None  # type: ignore
  _HAS_PARAMS = False


def _clip(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else (hi if x > hi else x)


def pscm_response_at(v_ego: float, fingerprint: Optional[str],
                     live_scale: float = 1.0) -> Tuple[float, float, float]:
  """Return (α, β, γ) for the speed bin containing v_ego, with β and γ
  scaled by (platform_scale × live_scale). α is not scaled because it is
  the worst-identified regression coefficient (R²=-0.17 on c0-only data)
  and was driving slight over-delivery in field tests when scaled."""
  base = FORD_PSCM_PLATFORM_SCALE.get(fingerprint, FORD_PSCM_PLATFORM_SCALE_DEFAULT)
  scale = base * live_scale
  a = b = g = 0.0
  for v_max, av, bv, gv, _r2 in FORD_PSCM_REGRESSION:
    if v_ego < v_max:
      a, b, g = av, bv, gv
      break
  else:
    _, a, b, g, _ = FORD_PSCM_REGRESSION[-1]
  return a, b * scale, g * scale


def bal_encode(desired_curvature: float, v_ego: float,
               fingerprint: Optional[str], live_scale: float = 1.0
               ) -> Tuple[float, float, float]:
  """Compute (c0, c1, c2) such that the PSCM polynomial delivers
  desired_curvature at the wheels. Uses c1+c2 only at equal utilization
  until u_c12 ≥ 1, then brings c0 into the budget as overflow.

  Returns the values in INTERNAL sign convention (positive desk → positive
  c1). The carcontroller negates before packing onto CAN."""
  a_v, b_v, g_v = pscm_response_at(v_ego, fingerprint, live_scale)
  if abs(desired_curvature) < 1e-9:
    return 0.0, 0.0, 0.0
  s = 1.0 if desired_curvature > 0.0 else -1.0

  denom_c12 = ((b_v * FORD_BAL_LC1) if b_v > 0.0 else 0.0) + \
              ((g_v * FORD_BAL_LC2) if g_v > 0.0 else 0.0)
  if denom_c12 > 1e-9 and abs(desired_curvature) <= denom_c12:
    u = abs(desired_curvature) / denom_c12
    c1 = s * u * FORD_BAL_LC1 if b_v > 0.0 else 0.0
    c2 = s * u * FORD_BAL_LC2 if g_v > 0.0 else 0.0
    return 0.0, c1, c2

  # Overflow path: include c0 to extend authority at the ceiling
  denom_full = ((a_v * FORD_BAL_LC0) if a_v > 0.0 else 0.0) + \
               ((b_v * FORD_BAL_LC1) if b_v > 0.0 else 0.0) + \
               ((g_v * FORD_BAL_LC2) if g_v > 0.0 else 0.0)
  if denom_full <= 1e-9:
    return 0.0, 0.0, 0.0
  u = min(1.0, abs(desired_curvature) / denom_full)
  c0 = s * u * FORD_BAL_LC0 if a_v > 0.0 else 0.0
  c1 = s * u * FORD_BAL_LC1 if b_v > 0.0 else 0.0
  c2 = s * u * FORD_BAL_LC2 if g_v > 0.0 else 0.0
  return c0, c1, c2


class FordBalLiveScale:
  """Always-on online estimator. Observes (desk, act_κ) on stable curves,
  accumulates per-speed-bucket gain estimates, and slowly nudges a live
  multiplier toward 1/measured_gain. Bounded ±15% from the platform default.

  Persistence via openpilot Params if available; in-memory otherwise.
  Restored on init keyed by (fingerprint, version) — value is discarded if
  either changes."""
  def __init__(self):
    self._params = Params() if _HAS_PARAMS else None
    self._fingerprint: Optional[str] = None
    self._live = 1.0
    # Ramping filter decay (torqued pattern): converges fast early, slows
    # as data accumulates so a single bad route can't yank a converged value.
    self._decay = FORD_BAL_LIVE_MIN_DECAY
    n = len(FORD_BAL_LIVE_BUCKETS_V) - 1
    self._bucket_act_sum = [0.0] * n
    self._bucket_desk_sum = [0.0] * n
    self._bucket_count = [0] * n
    self._desk_buf: list[float] = []
    self._frame = 0
    self._persist_frame = 0

  # --- persistence ---------------------------------------------------------
  def _restore(self, fingerprint: str) -> None:
    """Restore (live, decay) from Params. Payload format:
        "FINGERPRINT:VERSION:LIVE_VALUE:DECAY"
    Older 3-field payloads (no decay) are accepted; decay defaults to MIN."""
    self._fingerprint = fingerprint
    self._live = 1.0
    self._decay = FORD_BAL_LIVE_MIN_DECAY
    if self._params is None:
      return
    try:
      raw = self._params.get("FordBalLiveScale")
      if not raw:
        return
      text = raw.decode() if isinstance(raw, bytes) else raw
      parts = text.split(":")
      if len(parts) < 3:
        return
      fp_s, ver_s, val_s = parts[0], parts[1], parts[2]
      if fp_s != fingerprint or ver_s != FORD_BAL_LIVE_VERSION:
        return
      self._live = _clip(float(val_s),
                         FORD_BAL_LIVE_BOUND_LOW,
                         FORD_BAL_LIVE_BOUND_HIGH)
      if len(parts) >= 4:
        self._decay = _clip(float(parts[3]),
                            FORD_BAL_LIVE_MIN_DECAY,
                            FORD_BAL_LIVE_MAX_DECAY)
    except Exception:
      self._live = 1.0
      self._decay = FORD_BAL_LIVE_MIN_DECAY

  def _persist(self) -> None:
    if self._params is None or self._fingerprint is None:
      return
    try:
      payload = (f"{self._fingerprint}:{FORD_BAL_LIVE_VERSION}:"
                 f"{self._live:.4f}:{self._decay:.1f}")
      self._params.put_nonblocking("FordBalLiveScale", payload)
    except Exception:
      # UnknownKeyName if host openpilot hasn't registered the key — that's
      # OK, the estimator continues running in-memory.
      pass

  # --- public ---------------------------------------------------------------
  def current_scale(self, fingerprint: Optional[str]) -> float:
    if fingerprint is not None and fingerprint != self._fingerprint:
      self._restore(fingerprint)
    return self._live

  def update(self, desk: float, act_k: float, v: float,
             lat_active: bool, steering_pressed: bool) -> None:
    """Tick once per carcontroller frame. Cheap when the gates reject
    (most non-curve frames)."""
    self._frame += 1
    self._desk_buf.append(desk)
    if len(self._desk_buf) > FORD_BAL_LIVE_DESK_BUF:
      self._desk_buf.pop(0)

    # Persistence throttle always fires while running (independent of gate
    # pass rate) so a value gets written even on routes with few curves.
    if self._frame - self._persist_frame >= FORD_BAL_LIVE_PERSIST_FRAMES:
      self._persist_frame = self._frame
      self._persist()

    # Frame gates
    if not lat_active or steering_pressed:
      return
    if v < FORD_BAL_LIVE_V_MIN:
      return
    if abs(desk) < FORD_BAL_LIVE_DESK_MIN:
      return
    if len(self._desk_buf) < FORD_BAL_LIVE_DESK_BUF:
      return
    # Stdev gate — reject volatile desk (planner is hunting / lane change)
    mean = sum(self._desk_buf) / len(self._desk_buf)
    var = sum((x - mean) ** 2 for x in self._desk_buf) / len(self._desk_buf)
    if var > FORD_BAL_LIVE_DESK_VAR_MAX * FORD_BAL_LIVE_DESK_VAR_MAX:
      return

    # Bucket the sample
    bucket = None
    for i in range(len(FORD_BAL_LIVE_BUCKETS_V) - 1):
      lo = FORD_BAL_LIVE_BUCKETS_V[i]
      hi = FORD_BAL_LIVE_BUCKETS_V[i + 1]
      if lo <= v < hi:
        bucket = i
        break
    if bucket is None:
      return
    self._bucket_act_sum[bucket] += act_k * (1.0 if desk > 0 else -1.0)
    self._bucket_desk_sum[bucket] += abs(desk)
    self._bucket_count[bucket] += 1

    # Update the live scale when enough data has accumulated
    total = sum(self._bucket_count)
    populated = sum(1 for c in self._bucket_count if c >= FORD_BAL_LIVE_BUCKET_MIN_PT)
    if total >= FORD_BAL_LIVE_MIN_FRAMES and populated >= 1:
      gains = []
      for i in range(len(self._bucket_count)):
        if self._bucket_count[i] >= FORD_BAL_LIVE_BUCKET_MIN_PT and self._bucket_desk_sum[i] > 1e-6:
          gains.append(self._bucket_act_sum[i] / self._bucket_desk_sum[i])
      if gains:
        mean_gain = sum(gains) / len(gains)
        if 0.5 < mean_gain < 2.0:  # sanity
          target = _clip(self._live / max(mean_gain, 0.5),
                         FORD_BAL_LIVE_BOUND_LOW,
                         FORD_BAL_LIVE_BOUND_HIGH)
          # Ramping first-order filter: alpha = 1/decay, decay grows over
          # time so each successive update moves the live value less.
          alpha = 1.0 / self._decay
          self._live = (1.0 - alpha) * self._live + alpha * target
          self._live = _clip(self._live,
                             FORD_BAL_LIVE_BOUND_LOW,
                             FORD_BAL_LIVE_BOUND_HIGH)
          self._decay = min(self._decay + FORD_BAL_LIVE_DECAY_STEP,
                            FORD_BAL_LIVE_MAX_DECAY)
        # Decay buckets 25% so stale data doesn't dominate
        for i in range(len(self._bucket_count)):
          self._bucket_act_sum[i] *= 0.75
          self._bucket_desk_sum[i] *= 0.75
          self._bucket_count[i] = int(self._bucket_count[i] * 0.75)
