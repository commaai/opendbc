"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser


class CarStateExt:
  def __init__(self):
    super().__init__()

    self.aBasis = 0.0

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

  def update_canfd_ext(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.aBasis = cp.vl["TCS"]["aBasis"]
