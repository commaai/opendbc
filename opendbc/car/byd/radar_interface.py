#!/usr/bin/env python3
# BYD front radar: same framing pattern as toyota/honda/gm (single update(), CANParser, trigger frame).
# Unlike those stacks, BYD DBC exposes VLEAD as absolute lead speed; radard expects RadarPoint.vRel
# relative to ego, so we parse WHEEL_SPEED on the PT bus (same wheel scaling as CarState) for v_ego.
from collections import deque

from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.structs import RadarData
from opendbc.car.byd.values import CANBUS, DBC, CAR

RADAR_START_ADDR = 0x280
RADAR_MSG_COUNT = 10
SEAL6_RADAR_START_ADDR = 0x670
SEAL6_RADAR_MSG_COUNT = 16
RADAR_FREQ_HZ = 20
BUMPER_OFFSET_M = 4.0
SEAL6_INVALID_LONG_DIST = 0.5
# Max publish range (dRel). Was 120m; stock radar decodes real in-lane leads to ~105m on highway logs.
SEAL6_DREL_MAX = 160.0
# Empty slots decode as LONG_DIST ~183–636 m — hard reject at/above this raw distance.
SEAL6_GHOST_LONG_DIST_MIN = 175.0
# ~144m cluster correlates with vision ~20m (route 2026-07-17--01-30-08); not real far leads.
SEAL6_SENTINEL_LONG_MIN = 138.0
SEAL6_SENTINEL_LONG_MAX = 153.0
SEAL6_YREL_ABS_MAX = 1.2
# Stricter gates above this dRel to limit phantom far tracks while extending range.
SEAL6_FAR_DREL_M = 100.0
SEAL6_FAR_YREL_ABS_MAX = 0.85
SEAL6_FAR_VALID_CNT_ON = 8
# VLEAD_KPH @ bit 112 is NOT real speed: stock logs show ~71% stuck at ~117.8 kph and
# ~3% at ~25.6 kph sentinels (route 2026-07-17--01-30-08). Using it as absolute speed
# created fake vRel≈-13 and phantom brakes. Derive vRel from LONG_DIST rate instead.
SEAL6_VREL_LP_ALPHA = 0.35
SEAL6_RANGE_RATE_DT = 1.0 / RADAR_FREQ_HZ
SEAL6_VREL_RESET_JUMP = 25.0  # m/s; slot reuse / teleport
# Low-speed stop-and-go: radard may prefer a radar-only frozen lead over a moving vision lead.
# Without vision here, reject stale stationary ghosts (frozen dRel + vRel≈0) and range snaps.
SEAL6_LOW_SPEED_V_EGO_MAX = 4.0   # match radard V_EGO_STATIONARY
SEAL6_LOW_SPEED_FROZEN_EPS = 0.08
SEAL6_LOW_SPEED_FROZEN_FRAMES = 8   # 0.4s @ 20Hz
SEAL6_LOW_SPEED_EGO_CREEP = 0.05    # m/s; any rollout, not just highway creep
SEAL6_LOW_SPEED_VREL_STATIONARY = 0.5
SEAL6_LOW_SPEED_SNAP_M = 1.25       # m; single-frame range jump with no vRel
SEAL6_LOW_SPEED_CLOSE_DREL = 25.0
SEAL6_LOW_SPEED_CLOSE_VALID_CNT = 10  # 0.5s @ 20Hz before publishing close tracks

# --- Temporal filtering / hysteresis ---
# Aggressive persistence to prevent track churn and Kalman filter resets
# Higher requirements for stability reduce false track switches
CONF_ON = 0.75         # Reasonable confidence threshold
CONF_OFF = 0.50
VALID_CNT_ON = 5       # Require 5 frames to be stable (increased from 3 to prevent track churn)
# Keep tracks alive for much longer to prevent Kalman filter resets
# When tracks briefly fail filtering, keep them published to maintain continuity
# Delete tracks after 15 missed frames; keeps tracks alive ~750ms (15*50ms) when filtering fails
MISS_MAX = 15

# --- Plausibility gates ---
DREL_MIN = 0.75        # near-field radar ghosts
DREL_MAX = 200.0
# Filter side-lane vehicles to prevent phantom braking
# Balanced at 1.5m - allows legitimate vehicles while filtering obvious side-lane vehicles
# Typical lane width ~3.5m, so 1.5m = center ~43% (reasonable for lane keeping)
YREL_ABS_MAX = 1.5     # Balanced: filter obvious side-lane vehicles (>1.5m) while allowing legitimate vehicles
VREL_ABS_MAX = 60.0
AREL_ABS_MAX = 12.0


def _create_radar_can_parser(CP):
  dbc = DBC[CP.carFingerprint]
  if Bus.radar not in dbc:
    return None

  if CP.carFingerprint == CAR.BYD_SEAL6:
    msg_count = SEAL6_RADAR_MSG_COUNT
  else:
    msg_count = RADAR_MSG_COUNT

  messages = [(f"RADAR_TRACK_{i:02d}", RADAR_FREQ_HZ) for i in range(msg_count)]
  return CANParser(dbc[Bus.radar], messages, CANBUS.radar_bus)


class RadarInterface(RadarInterfaceBase):
  """BYD radar parser with confidence and persistence filtering."""

  def __init__(self, CP):
    super().__init__(CP)

    self.seal6_radar = CP.carFingerprint == CAR.BYD_SEAL6
    self.radar_msg_count = SEAL6_RADAR_MSG_COUNT if self.seal6_radar else RADAR_MSG_COUNT
    radar_start = SEAL6_RADAR_START_ADDR if self.seal6_radar else RADAR_START_ADDR

    self.updated_messages = set()
    self.trigger_msg = radar_start + self.radar_msg_count - 1

    self.track_id = 0
    self.slot_track_id = {}     # slot -> stable trackId

    self.valid_cnt = {i: 0 for i in range(self.radar_msg_count)}
    self.miss_cnt = {i: 0 for i in range(self.radar_msg_count)}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)

    # Ego speed on PT bus for vRel = v_lead_radar - v_ego (same wheel scaling as CarState)
    self.speed_cp = None
    if not CP.radarUnavailable and Bus.pt in DBC[CP.carFingerprint]:
      self.speed_cp = CANParser(DBC[CP.carFingerprint][Bus.pt], [("WHEEL_SPEED", 50)], CANBUS.main_bus)

    # Track persistence: store last known position/velocity for disappeared tracks
    # This helps match reappearing tracks to their previous track IDs
    # Key: track_id, Value: (dRel, yRel, vRel, miss_frames)
    self.track_history: dict[int, tuple[float, float, float, int]] = {}
    self.MAX_HISTORY_FRAMES = 10  # Keep history for up to 10 frames after track disappears

    # Seal6: per-slot range history for vRel (VLEAD_KPH is unusable)
    self.seal6_prev_drel: dict[int, float] = {}
    self.seal6_vrel_filt: dict[int, float] = {}
    self.seal6_drel_ring: dict[int, deque[float]] = {}

  def _ego_speed_accel(self) -> tuple[float, float]:
    if self.speed_cp is None:
      return 0.0, 0.0
    vl = self.speed_cp.vl["WHEEL_SPEED"]
    fl = float(vl["WHEELSPEED_FL"])
    fr = float(vl["WHEELSPEED_FR"])
    rl = float(vl["WHEELSPEED_BL"])
    rr = float(vl["WHEELSPEED_BR"])
    v_ego = (fl + fr + rl + rr) / 4.0 * CV.KPH_TO_MS * self.CP.wheelSpeedFactor
    return v_ego, 0.0

  def _seal6_long_dist_plausible(self, long_dist: float) -> bool:
    if long_dist <= SEAL6_INVALID_LONG_DIST:
      return False
    if long_dist >= SEAL6_GHOST_LONG_DIST_MIN:
      return False
    if SEAL6_SENTINEL_LONG_MIN <= long_dist <= SEAL6_SENTINEL_LONG_MAX:
      return False
    return long_dist <= (SEAL6_DREL_MAX + BUMPER_OFFSET_M)

  def _seal6_vrel_from_range(self, slot: int, dRel: float) -> float:
    """Estimate vRel from dRel finite difference; ignore unusable VLEAD_KPH sentinels."""
    prev = self.seal6_prev_drel.get(slot)
    self.seal6_prev_drel[slot] = dRel
    if prev is None:
      self.seal6_vrel_filt[slot] = 0.0
      return 0.0

    raw = (dRel - prev) / SEAL6_RANGE_RATE_DT
    if abs(raw) > SEAL6_VREL_RESET_JUMP:
      self.seal6_vrel_filt[slot] = 0.0
      return 0.0

    prev_filt = self.seal6_vrel_filt.get(slot, raw)
    filt = SEAL6_VREL_LP_ALPHA * raw + (1.0 - SEAL6_VREL_LP_ALPHA) * prev_filt
    self.seal6_vrel_filt[slot] = filt
    return float(filt)

  def _seal6_low_speed_untrusted(self, slot: int, dRel: float, yRel: float, vRel: float, v_ego: float, prev_drel: float | None) -> bool:
    """Seal6 stop-and-go: drop radar tracks vision would reject (frozen stationary / range snap)."""
    if v_ego >= SEAL6_LOW_SPEED_V_EGO_MAX:
      return False
    if abs(yRel) > 0.5 or not (DREL_MIN <= dRel <= SEAL6_LOW_SPEED_CLOSE_DREL):
      return False

    if prev_drel is not None:
      if abs(dRel - prev_drel) > SEAL6_LOW_SPEED_SNAP_M and abs(vRel) < 1.0:
        return True

    ring = self.seal6_drel_ring.setdefault(slot, deque(maxlen=SEAL6_LOW_SPEED_FROZEN_FRAMES))
    ring.append(dRel)
    if len(ring) >= SEAL6_LOW_SPEED_FROZEN_FRAMES:
      frozen = max(ring) - min(ring) < SEAL6_LOW_SPEED_FROZEN_EPS
      if frozen and abs(vRel) < SEAL6_LOW_SPEED_VREL_STATIONARY and v_ego > SEAL6_LOW_SPEED_EGO_CREEP:
        return True

    return False

  def _read_track(self, msg, v_ego, a_ego, slot: int | None = None):
    if self.seal6_radar:
      long_dist = float(msg.get("LONG_DIST", 0.0))
      track_id = int(msg.get("TRACK_ID", 0))
      # Empty/ghost slots: invalid LONG_DIST, known sentinel bands, or TRACK_ID=0.
      meas_ok = track_id > 0 and self._seal6_long_dist_plausible(long_dist)
      conf = 0.95 if meas_ok else 0.0
      lat_dist = float(msg.get("LAT_DIST", 0.0))
      dRel = long_dist - BUMPER_OFFSET_M
      yRel = lat_dist
      # Do not use VLEAD_KPH — stock DBC field is sentinel garbage (see SEAL6_* comments).
      if slot is not None and meas_ok:
        vRel = self._seal6_vrel_from_range(slot, dRel)
      else:
        vRel = 0.0
      aRel = 0.0
      return conf, meas_ok, dRel, yRel, vRel, aRel

    long_dist = float(msg.get("LONG_DIST", 255.0))
    meas_ok = long_dist < 255.0
    conf = float(msg.get("CONFIDENCE", 0.0))
    lat_dist = float(msg.get("LAT_DIST", 0.0))
    vlead = float(msg.get("VLEAD", 0.0))
    alead = float(msg.get("ALEAD", 0.0))

    dRel = long_dist - BUMPER_OFFSET_M
    yRel = lat_dist
    vRel = vlead - v_ego
    aRel = alead - a_ego
    return conf, meas_ok, dRel, yRel, vRel, aRel

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.speed_cp is not None:
      self.speed_cp.update(can_strings)
    v_ego, a_ego = self._ego_speed_accel()

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages, v_ego, a_ego)
    self.updated_messages.clear()
    return rr

  def _find_matching_track_id(self, dRel, yRel, vRel):
    """
    Try to match a reappearing track to a previous track ID based on position/velocity similarity.
    This prevents track ID churn when tracks briefly disappear and reappear.
    """
    best_match_id = None
    best_distance = float('inf')

    # Match thresholds: allow some drift for brief dropouts
    # More lenient thresholds to better match reappearing tracks
    MAX_DIST_DIFF = 5.0   # meters (allows for movement during dropout)
    MAX_Y_DIFF = 2.0      # meters (lateral drift tolerance - increased from 1.5)
    MAX_V_DIFF = 8.0      # m/s (velocity change tolerance - increased from 5.0)

    for track_id, (hist_d, hist_y, hist_v, miss_frames) in self.track_history.items():
      # Only match tracks that disappeared recently (within last few frames)
      if miss_frames > 8:  # Increased from 5 to allow matching slightly older tracks
        continue

      dist_diff = abs(hist_d - dRel)
      y_diff = abs(hist_y - yRel)
      v_diff = abs(hist_v - vRel)

      # Check if within thresholds
      if dist_diff <= MAX_DIST_DIFF and y_diff <= MAX_Y_DIFF and v_diff <= MAX_V_DIFF:
        # Use weighted distance (distance most important, then lateral, then velocity)
        # Reduced weight on velocity since it can change more during brief dropouts
        distance = dist_diff * 2.0 + y_diff * 1.5 + v_diff * 0.3  # Reduced v_diff weight from 0.5
        if distance < best_distance:
          best_distance = distance
          best_match_id = track_id

    return best_match_id

  def _alloc_track_id(self, slot, dRel=None, yRel=None, vRel=None):
    """
    Allocate track ID for a slot. Try to match to previous track ID if this is a reappearing track.
    Priority: 1) Reuse existing slot mapping, 2) Match to history, 3) Allocate new ID
    """
    # If slot already has a track ID mapping, reuse it (most common case)
    # This prevents track ID churn when tracks temporarily disappear and reappear
    if slot in self.slot_track_id:
      track_id = self.slot_track_id[slot]
      # If this track ID is in history (was recently deleted), remove it from history
      if track_id in self.track_history:
        self.track_history.pop(track_id, None)
      return track_id

    # Slot doesn't have a mapping - try to match to history based on position
    if dRel is not None and yRel is not None and vRel is not None:
      matched_id = self._find_matching_track_id(dRel, yRel, vRel)
      if matched_id is not None:
        self.slot_track_id[slot] = matched_id
        # Remove from history since we matched it
        self.track_history.pop(matched_id, None)
        return matched_id

    # No match found - allocate new track ID
    self.slot_track_id[slot] = self.track_id
    self.track_id += 1
    return self.slot_track_id[slot]

  def _kill_slot(self, slot):
    # Before deleting, save track state to history for matching reappearing tracks
    if slot in self.pts:
      pt = self.pts[slot]
      track_id = pt.trackId
      # Save last known state to history
      self.track_history[track_id] = (pt.dRel, pt.yRel, pt.vRel, 0)
      del self.pts[slot]

    self.valid_cnt[slot] = 0
    self.miss_cnt[slot] = 0
    self.seal6_prev_drel.pop(slot, None)
    self.seal6_vrel_filt.pop(slot, None)
    self.seal6_drel_ring.pop(slot, None)
    # Keep slot_track_id to allow matching when track reappears
    # Don't delete it here - let it persist for potential rematching

  def _update(self, updated_messages, v_ego: float = 0.0, a_ego: float = 0.0):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for slot in range(self.radar_msg_count):
      msg = self.rcp.vl[f"RADAR_TRACK_{slot:02d}"]
      prev_drel = self.seal6_prev_drel.get(slot) if self.seal6_radar else None

      conf, meas_ok, dRel, yRel, vRel, aRel = self._read_track(msg, v_ego, a_ego, slot)

      low_speed_untrusted = False
      if self.seal6_radar and meas_ok:
        low_speed_untrusted = self._seal6_low_speed_untrusted(slot, dRel, yRel, vRel, v_ego, prev_drel)
        if low_speed_untrusted:
          meas_ok = False
          conf = 0.0
          self._kill_slot(slot)
          continue

      if self.seal6_radar:
        drel_max = SEAL6_DREL_MAX
        yrel_base = SEAL6_YREL_ABS_MAX
        if dRel > SEAL6_FAR_DREL_M:
          yrel_base = min(yrel_base, SEAL6_FAR_YREL_ABS_MAX)
      else:
        drel_max = DREL_MAX
        yrel_base = YREL_ABS_MAX
      yrel_max = yrel_base if dRel <= 30.0 else yrel_base * 0.7

      valid_cnt_on = VALID_CNT_ON
      if self.seal6_radar and dRel > SEAL6_FAR_DREL_M:
        valid_cnt_on = SEAL6_FAR_VALID_CNT_ON
      elif self.seal6_radar and v_ego < SEAL6_LOW_SPEED_V_EGO_MAX and abs(yRel) <= 0.5 and dRel <= SEAL6_LOW_SPEED_CLOSE_DREL:
        valid_cnt_on = max(valid_cnt_on, SEAL6_LOW_SPEED_CLOSE_VALID_CNT)

      if self.seal6_radar:
        plausible = (
          meas_ok and
          (DREL_MIN <= dRel <= drel_max) and
          (abs(yRel) <= yrel_max) and
          (abs(vRel) <= VREL_ABS_MAX)
        )
      else:
        plausible = (
          meas_ok and
          (DREL_MIN <= dRel <= drel_max) and
          (abs(yRel) <= yrel_max) and
          (abs(vRel) <= VREL_ABS_MAX) and
          (abs(aRel) <= AREL_ABS_MAX)
        )

      # Require higher confidence for vehicles near the edge (even if within threshold)
      # This adds extra filtering for vehicles that are close to the lateral limit
      conf_required = CONF_ON
      if abs(yRel) > 1.0:  # More than 1.0m to side (67% of 1.5m threshold)
        conf_required = max(CONF_ON, 0.85)  # Require higher confidence (85%)

      good = plausible and (conf >= conf_required)

      if good:
        self.valid_cnt[slot] += 1
        self.miss_cnt[slot] = 0
      else:
        if (not plausible) or (conf <= CONF_OFF) or (not meas_ok):
          self.miss_cnt[slot] += 1
        else:
          self.valid_cnt[slot] = max(self.valid_cnt[slot] - 1, 0)

      # Check if track is moving away (should be deleted immediately, don't publish)
      track_moving_away = False
      if slot in self.pts:
        pt = self.pts[slot]
        # Track is moving laterally away beyond threshold - delete immediately
        if abs(yRel) > yrel_max * 1.2:  # 20% beyond threshold (1.8m for close, 1.26m for far)
          track_moving_away = True
        # Track is moving forward away rapidly (dRel increasing, positive vRel)
        elif meas_ok and dRel > pt.dRel + 3.0 and vRel > 2.0:  # Moved >3m forward and moving away >2m/s
          track_moving_away = True
        # Track was previously good but now clearly side-lane (moved beyond threshold)
        elif abs(yRel) > yrel_max and self.valid_cnt[slot] >= valid_cnt_on:
          # Was a valid track but now moved to side-lane - delete immediately
          track_moving_away = True

      # Don't publish if track is moving away
      publish = (self.valid_cnt[slot] >= valid_cnt_on) and not track_moving_away
      # Seal 6: never overwrite a validated track with out-of-range ghost LONG_DIST.
      if self.seal6_radar:
        publish = publish and good

      # Keep tracks alive longer - publish them even when they briefly fail filtering
      # This prevents Kalman filter resets in radard.py
      # BUT: Don't keep tracks alive if they're clearly moving away
      keep_alive = False
      if slot in self.pts and not track_moving_away:
        # Track already exists - keep it alive if it hasn't exceeded MISS_MAX
        if self.miss_cnt[slot] <= MISS_MAX:
          keep_alive = True

      if publish:
        if slot not in self.pts:
          self.pts[slot] = RadarData.RadarPoint()
          # Try to match to previous track ID when allocating
          self.pts[slot].trackId = self._alloc_track_id(slot, dRel, yRel, vRel)
        else:
          # Track exists - remove from history if it was there
          track_id = self.pts[slot].trackId
          self.track_history.pop(track_id, None)

        pt = self.pts[slot]
        pt.measured = True
        pt.dRel = dRel
        pt.yRel = yRel
        pt.vRel = vRel
        pt.aRel = aRel
        pt.yvRel = float("nan")

      elif keep_alive:
        # Track exists but filtering failed temporarily - keep it alive
        # Continue updating with current radar values (even if noisy) to maintain Kalman filter continuity
        # The Kalman filter in radard.py can handle some noise, but needs continuous input
        pt = self.pts[slot]
        # Update with current values but mark as unmeasured to indicate lower reliability
        # This prevents track deletion while still providing input to the Kalman filter
        pt.measured = False
        # Still update with current radar values (slightly smoothed) for continuity
        if meas_ok and plausible:  # Use values if measurement is OK, even if confidence is low
          # Use exponential smoothing to reduce noise when confidence is low
          alpha = 0.7  # Smoothing factor (higher = more weight to new value)
          pt.dRel = alpha * dRel + (1 - alpha) * pt.dRel
          pt.yRel = alpha * yRel + (1 - alpha) * pt.yRel
          pt.vRel = alpha * vRel + (1 - alpha) * pt.vRel
          pt.aRel = alpha * aRel + (1 - alpha) * pt.aRel
        # If measurement is not OK, keep last known values (already set)

      else:
        # Track doesn't exist or has exceeded MISS_MAX, or is moving away
        if track_moving_away:
          # Track is moving away - delete immediately (don't wait for MISS_MAX)
          self._kill_slot(slot)
          # DON'T delete slot_track_id mapping here - keep it for potential rematching
          # The slot_track_id will be reused if the track reappears in the same slot
        elif self.miss_cnt[slot] > MISS_MAX:
          # Track exceeded miss count - delete it
          self._kill_slot(slot)
          # DON'T delete slot_track_id mapping here - keep it for potential rematching
        elif slot in self.pts:
          # Should not reach here if keep_alive logic is correct, but keep as fallback
          self.pts[slot].measured = False

    # Update history for all existing tracks (increment miss frames) - do this once per frame
    # Clean up old history entries
    tracks_to_remove = []
    for track_id, (hist_d, hist_y, hist_v, miss_frames) in list(self.track_history.items()):
      # Increment miss frames for tracks in history
      new_miss_frames = miss_frames + 1
      self.track_history[track_id] = (hist_d, hist_y, hist_v, new_miss_frames)
      # Remove if too old
      if new_miss_frames > self.MAX_HISTORY_FRAMES:
        tracks_to_remove.append(track_id)

    for track_id in tracks_to_remove:
      self.track_history.pop(track_id, None)

    ret.points = list(self.pts.values())
    return ret
