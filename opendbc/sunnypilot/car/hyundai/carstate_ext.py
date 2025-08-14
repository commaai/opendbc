"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser], speed_conv: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False
      ret.cruiseState.speed = 255
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False

      if not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_NO_FCA:
        cp_cruise = cp if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_RADAR_FCA else cp_cam

        aeb_src = "FCA11"
        aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
        aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src]["FCA_CmdAct"] != 0
        ret.stockFcw = aeb_warning and not aeb_braking
        ret.stockAeb = aeb_warning and aeb_braking

  def update_canfd_ext(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.aBasis = cp.vl["TCS"]["aBasis"]
