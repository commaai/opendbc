"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class HondaFlagsSP(IntFlag):
  CLARITY = 1
  EPS_MODIFIED = 2


class HondaSafetyFlagsSP:
  CLARITY = 1
  GAS_INTERCEPTOR = 2
