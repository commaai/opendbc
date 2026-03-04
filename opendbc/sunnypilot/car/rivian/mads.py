"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections import namedtuple

from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase

MAX_STEERING_ANGLE = 90.0

MadsDataSP = namedtuple("MadsDataSP",
                        ["lka_icon_states", "lat_active"])


class MadsCarController:
  def __init__(self):
    self.mads = MadsDataSP(False, False)

    self.lka_icon_states = False
    self.lat_active = False

  def mads_status_update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS: CarStateBase) -> MadsDataSP:
    if CC_SP.mads.available:
      self.lka_icon_states = self.lat_active
      self.lat_active = CC.latActive and abs(CS.out.steeringAngleDeg) < MAX_STEERING_ANGLE
    else:
      self.lka_icon_states = CC.enabled
      self.lat_active = CC.latActive

    return MadsDataSP(self.lka_icon_states, self.lat_active)

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS: CarStateBase) -> None:
    self.mads = self.mads_status_update(CC, CC_SP, CS)
