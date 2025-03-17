"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus,structs

from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.prev_lkas_button = self.lkas_button
    self.lkas_button = cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"]
