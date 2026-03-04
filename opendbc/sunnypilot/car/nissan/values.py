"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections import namedtuple
from opendbc.car import structs


class NissanSafetyFlagsSP:
  DEFAULT = 0
  LEAF = 1


ButtonType = structs.CarState.ButtonEvent.Type
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


BUTTONS = [
  Button(ButtonType.accelCruise, "CRUISE_THROTTLE", "RES_BUTTON", [1]),
  Button(ButtonType.decelCruise, "CRUISE_THROTTLE", "SET_BUTTON", [1]),
]
