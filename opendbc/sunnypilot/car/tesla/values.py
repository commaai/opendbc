"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntFlag


class TeslaFlagsSP(IntFlag):
  HAS_VEHICLE_BUS = 1  # 3-finger infotainment press signal is present on the VEHICLE bus with the deprecated Tesla harness installed


class TeslaSafetyFlagsSP:
  HAS_VEHICLE_BUS = 1
