"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

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

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

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
