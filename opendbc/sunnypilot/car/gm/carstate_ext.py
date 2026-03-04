"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.gm.values_ext import GMFlagsSP


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    pt_cp = can_parsers[Bus.pt]

    if self.CP_SP.flags & GMFlagsSP.NON_ACC:
      ret.cruiseState.enabled = pt_cp.vl["ECMCruiseControl"]["CruiseActive"] != 0
      ret.cruiseState.speed = pt_cp.vl["ECMCruiseControl"]["CruiseSetSpeed"] * CV.KPH_TO_MS
      ret.accFaulted = False
