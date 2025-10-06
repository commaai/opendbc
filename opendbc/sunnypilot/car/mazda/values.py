"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections import namedtuple

from opendbc.car import structs

ButtonType = structs.CarState.ButtonEvent.Type
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

BUTTONS = [
  Button(ButtonType.accelCruise, "CRZ_BTNS", "SET_P", [1]),
  Button(ButtonType.decelCruise, "CRZ_BTNS", "SET_M", [1]),
  Button(ButtonType.cancel, "CRZ_BTNS", "CAN_OFF", [1]),
  Button(ButtonType.resumeCruise, "CRZ_BTNS", "RES", [1]),
]
