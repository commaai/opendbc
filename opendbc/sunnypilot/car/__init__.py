"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
import json
ParamType = structs.CarControlSP.ParamType


def get_param_object(params: list[structs.CarControlSP.Param], key: str) -> structs.CarControlSP.Param | None:
  return next((param for param in params if param.key == key), None)


def get_param(params: list[structs.CarControlSP.Param], key: str, default_value = None) -> bytes | str | int | float | bool | dict:
  if (param := get_param_object(params, key)) is None:
    return default_value

  param_value = _get_param_as_type(param)
  if not isinstance(param_value, type(default_value)) and default_value is not None:
    raise ValueError(f"Param {key} has type {type(param_value)}, but we expected {type(default_value)} based on the default value")

  return param_value


def _get_param_as_type(param: structs.CarControlSP.Param, *, _forced_param_type: ParamType = None) -> bytes | str | int | float | bool | dict:
  """
  Convert a Param to its value in the right type
  :param param: The Param to convert.
  :param  _forced_param_type: (Only for easy sync, will be remove in the future) If given, it will be used instead of the param's own type.
  Must be explicitly passed as a keyword argument.
  """
  return _convert_param_to_type(param.value, _forced_param_type or param.type)


def _convert_param_to_type(value: bytes, param_type: ParamType) -> bytes | str | int | float | bool | dict:
  """
  Convert a byte value to the specified param type. Used internally when getting a Param to convert it to the right type.
  If this method looks familiar, it's because on SP we have a similar one in sunnylink/utils.py.
  """

  # We convert to string anything that isn't bytes first. We later transform further.
  if param_type != ParamType.bytes:
    value = value.decode('utf-8')  # type: ignore

  if param_type == ParamType.string:
    value = value
  elif param_type == ParamType.bool:
    value = value.lower() in ('true', '1', 'yes')  # type: ignore
  elif param_type == ParamType.int:
    value = int(value)  # type: ignore
  elif param_type == ParamType.float:
    value = float(value)  # type: ignore
  elif param_type == ParamType.time:
    value = str(value)  # type: ignore
  elif param_type == ParamType.json:
    value = json.loads(value)

  return value
