"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.honda.values import HONDA_BOSCH_RADARLESS


class MadsCarController:
  def __init__(self):
    self.dashed_lanes = False

  def update(self, CP: structs.CarParams, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    enable_mads = CC_SP.mads.available

    if enable_mads:
      self.dashed_lanes = CC_SP.mads.enabled and not CC.latActive
    else:
      self.dashed_lanes = CC.hudControl.lanesVisible if CP.carFingerprint in HONDA_BOSCH_RADARLESS else False
