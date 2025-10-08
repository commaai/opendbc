"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

TRAFFIC_SIGNAL_MAP = {
  1: "kph",
  36: "mph",
  65: "No overtake",
  66: "No overtake"
}

ZSS_DIFF_THRESHOLD = 4
ZSS_MAX_THRESHOLD = 10


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.acc_type = 1
    self.zss_compute = False
    self.zss_cruise_active_last = False
    self.zss_angle_offset = 0.
    self.zss_threshold_count = 0

    """Initialize traffic signal variables"""
    self._tsgn1 = None
    self._spdval1 = None
    self._splsgn1 = None
    self._tsgn2 = None
    self._splsgn2 = None
    self._tsgn3 = None
    self._splsgn3 = None
    self._tsgn4 = None
    self._splsgn4 = None

  @staticmethod
  def traffic_signal_description(tsgn):
    """Get description for traffic signal code"""
    desc = TRAFFIC_SIGNAL_MAP.get(int(tsgn))
    return f'{tsgn}: {desc}' if desc is not None else f'{tsgn}'

  def update_traffic_signals(self, cp_cam):
    """Update traffic signals with error handling"""
    try:
      # Add error handling for missing RSA messages
      tsgn1 = cp_cam.vl.get("RSA1", {}).get('TSGN1', 0)
      spdval1 = cp_cam.vl.get("RSA1", {}).get('SPDVAL1', 0)
      splsgn1 = cp_cam.vl.get("RSA1", {}).get('SPLSGN1', 0)
      tsgn2 = cp_cam.vl.get("RSA1", {}).get('TSGN2', 0)
      splsgn2 = cp_cam.vl.get("RSA1", {}).get('SPLSGN2', 0)
      tsgn3 = cp_cam.vl.get("RSA2", {}).get('TSGN3', 0)
      splsgn3 = cp_cam.vl.get("RSA2", {}).get('SPLSGN3', 0)
      tsgn4 = cp_cam.vl.get("RSA2", {}).get('TSGN4', 0)
      splsgn4 = cp_cam.vl.get("RSA2", {}).get('SPLSGN4', 0)
    except (KeyError, AttributeError) as e:
      # Handle case where RSA messages are not available
      carlog.debug(f"RSA messages not available: {e}")
      return

    has_changed = tsgn1 != self._tsgn1 \
                  or spdval1 != self._spdval1 \
                  or splsgn1 != self._splsgn1 \
                  or tsgn2 != self._tsgn2 \
                  or splsgn2 != self._splsgn2 \
                  or tsgn3 != self._tsgn3 \
                  or splsgn3 != self._splsgn3 \
                  or tsgn4 != self._tsgn4 \
                  or splsgn4 != self._splsgn4

    self._tsgn1 = tsgn1
    self._spdval1 = spdval1
    self._splsgn1 = splsgn1
    self._tsgn2 = tsgn2
    self._splsgn2 = splsgn2
    self._tsgn3 = tsgn3
    self._splsgn3 = splsgn3
    self._tsgn4 = tsgn4
    self._splsgn4 = splsgn4

    if not has_changed:
      return

    carlog.debug('---- TRAFFIC SIGNAL UPDATE -----')
    if tsgn1 is not None and tsgn1 != 0:
      carlog.debug(f'TSGN1: {self.traffic_signal_description(tsgn1)}')
    if spdval1 is not None and spdval1 != 0:
      carlog.debug(f'SPDVAL1: {spdval1}')
    if splsgn1 is not None and splsgn1 != 0:
      carlog.debug(f'SPLSGN1: {splsgn1}')
    if tsgn2 is not None and tsgn2 != 0:
      carlog.debug(f'TSGN2: {self.traffic_signal_description(tsgn2)}')
    if splsgn2 is not None and splsgn2 != 0:
      carlog.debug(f'SPLSGN2: {splsgn2}')
    if tsgn3 is not None and tsgn3 != 0:
      carlog.debug(f'TSGN3: {self.traffic_signal_description(tsgn3)}')
    if splsgn3 is not None and splsgn3 != 0:
      carlog.debug(f'SPLSGN3: {splsgn3}')
    if tsgn4 is not None and tsgn4 != 0:
      carlog.debug(f'TSGN4: {self.traffic_signal_description(tsgn4)}')
    if splsgn4 is not None and splsgn4 != 0:
      carlog.debug(f'SPLSGN4: {splsgn4}')
    carlog.debug('------------------------')

  def calculate_speed_limit(self):
    """Calculate speed limit from traffic signals with validation"""
    # Check all traffic sign slots for speed limits, not just tsgn1
    for tsgn, spdval in [(self._tsgn1, self._spdval1),
                         (self._tsgn2, None),
                         (self._tsgn3, None),
                         (self._tsgn4, None)]:

      if tsgn == 1 and spdval is not None and 0 < spdval <= 200:  # Reasonable speed range
        return spdval * CV.KPH_TO_MS

      if tsgn == 36 and spdval is not None and 0 < spdval <= 120:  # Reasonable MPH range
        return spdval * CV.MPH_TO_MS

    return 0

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP_SP.flags & ToyotaFlagsSP.SMART_DSU:
      self.acc_type = 1

    if self.CP_SP.enableGasInterceptor:
      gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = gas > 805

    # ZSS support thanks to zorrobyte, ErichMoraga, and dragonpilot
    if self.CP_SP.flags & ToyotaFlagsSP.ZSS:
      zorro_steer = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"]
      control_available = ret.cruiseState.available

      # Only compute ZSS offset when control is available
      if control_available and not self.zss_cruise_active_last:
        self.zss_threshold_count = 0
        self.zss_compute = True  # Control was just activated, so allow offset to be recomputed
      self.zss_cruise_active_last = control_available

      # Compute ZSS offset once we have meaningful angles
      if self.zss_compute and abs(ret.steeringAngleDeg) > 1e-3 and abs(zorro_steer) > 1e-3:
        self.zss_compute = False
        self.zss_angle_offset = zorro_steer - ret.steeringAngleDeg

      # Sanity checks
      steering_angle_deg = zorro_steer - self.zss_angle_offset
      if self.zss_threshold_count <= ZSS_MAX_THRESHOLD:
        if abs(ret.steeringAngleDeg - steering_angle_deg) > ZSS_DIFF_THRESHOLD:
          self.zss_threshold_count += 1
        else:
          ret.steeringAngleDeg = steering_angle_deg

    # Update traffic signals and speed limit
    self.update_traffic_signals(cp_cam)
    ret_sp.speedLimit = self.calculate_speed_limit()
