"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class ToyotaFlagsSP(IntFlag):
  SMART_DSU = 1
  RADAR_CAN_FILTER = 2
  ZSS = 4


class ToyotaSafetyFlagsSP:
  DEFAULT = 0
  UNSUPPORTED_DSU = 1
  GAS_INTERCEPTOR = 2
