"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections import namedtuple

from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

CoopSteeringDataSP = namedtuple("CoopSteeringDataSP",
                                ["control_type"])


class CoopSteeringCarController:
  def __init__(self):
    self.coop_steering = CoopSteeringDataSP(False)

  @staticmethod
  def coop_steering_status_update(CP_SP: structs.CarParamsSP) -> CoopSteeringDataSP:
    coop_steering = CP_SP.flags & TeslaFlagsSP.COOP_STEERING.value
    control_type = 2 if coop_steering else 1

    return CoopSteeringDataSP(control_type)

  def update(self, CP_SP: structs.CarParamsSP) -> None:
    self.coop_steering = self.coop_steering_status_update(CP_SP)
