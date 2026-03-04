"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs


class IntelligentCruiseButtonManagementInterfaceBase:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.CC_SP = None
    self.ICBM = None
    self.frame = 0
    self.button_frame = 0
    self.last_button_frame = 0
