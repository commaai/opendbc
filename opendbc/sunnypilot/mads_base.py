"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from abc import abstractmethod, ABC
from enum import StrEnum

from opendbc.car import structs
from opendbc.can.parser import CANParser


class MadsCarStateBase(ABC):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.lkas_button = 0
    self.prev_lkas_button = 0

  @abstractmethod
  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    pass
