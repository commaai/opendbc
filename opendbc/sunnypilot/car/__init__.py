"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs


def get_param(params: list[structs.CarControlSP.Param], key: str, default_key: str = None) -> str | None:
  return next((p.value for p in params if p.key == key), default_key)
