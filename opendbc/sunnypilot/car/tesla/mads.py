"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections import namedtuple

from opendbc.car import structs

MadsDataSP = namedtuple("MadsDataSP",
                        ["control_type"])


class MadsCarController:
  def __init__(self):
    super().__init__()
    self.mads = MadsDataSP(False)

  @staticmethod
  def mads_status_update(CC: structs.CarControl, CC_SP: structs.CarControlSP) -> MadsDataSP:
    mads_steering_only = CC_SP.mads.available and not CC.enabled
    control_type = 2 if mads_steering_only else 1

    return MadsDataSP(control_type)

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    self.mads = self.mads_status_update(CC, CC_SP)
