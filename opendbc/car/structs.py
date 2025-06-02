from dataclasses import dataclass as _dataclass, field, is_dataclass
from enum import Enum, StrEnum as _StrEnum, auto
from typing import dataclass_transform, get_origin

import os
import capnp
from opendbc.car.common.basedir import BASEDIR

# TODO: remove car from cereal/__init__.py and always import from opendbc
try:
  from cereal import car
except ImportError:
  capnp.remove_import_hook()
  car = capnp.load(os.path.join(BASEDIR, "car.capnp"))

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

# sunnypilot structs

AUTO_OBJ = object()


def auto_field():
  return AUTO_OBJ


@dataclass_transform()
def auto_dataclass(cls=None, /, **kwargs):
  cls_annotations = cls.__dict__.get('__annotations__', {})
  for name, typ in cls_annotations.items():
    current_value = getattr(cls, name)
    if current_value is AUTO_OBJ:
      origin_typ = get_origin(typ) or typ
      if isinstance(origin_typ, str):
        raise TypeError(f"Forward references are not supported for auto_field: '{origin_typ}'. Use a default_factory with lambda instead.")
      elif origin_typ in (int, float, str, bytes, list, tuple, bool) or is_dataclass(origin_typ):
        setattr(cls, name, field(default_factory=origin_typ))
      elif issubclass(origin_typ, Enum):  # first enum is the default
        setattr(cls, name, field(default=next(iter(origin_typ))))
      else:
        raise TypeError(f"Unsupported type for auto_field: {origin_typ}")

  # TODO: use slots, this prevents accidentally setting attributes that don't exist
  return _dataclass(cls, **kwargs)


class StrEnum(_StrEnum):
  @staticmethod
  def _generate_next_value_(name, *args):
    # auto() defaults to name.lower()
    return name


@auto_dataclass
class CarParamsSP:
  flags: int = auto_field()        # flags for car specific quirks
  safetyParam: int = auto_field()  # flags for custom safety flags

  neuralNetworkLateralControl: 'CarParamsSP.NeuralNetworkLateralControl' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl())

  @auto_dataclass
  class NeuralNetworkLateralControl:
    model: 'CarParamsSP.NeuralNetworkLateralControl.Model' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl.Model())
    fuzzyFingerprint: bool = auto_field()

    @auto_dataclass
    class Model:
      path: str = auto_field()
      name: str = auto_field()


@auto_dataclass
class ModularAssistiveDrivingSystem:
  state: 'ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState' = field(
    default_factory=lambda: ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.disabled
  )
  enabled: bool = auto_field()
  active: bool = auto_field()
  available: bool = auto_field()

  class ModularAssistiveDrivingSystemState(StrEnum):
    disabled = auto()
    paused = auto()
    enabled = auto()
    softDisabling = auto()
    overriding = auto()


@auto_dataclass
class CarControlSP:
  mads: 'ModularAssistiveDrivingSystem' = field(default_factory=lambda: ModularAssistiveDrivingSystem())
  params: list['CarControlSP.Param'] = auto_field()

  @auto_dataclass
  class Param:
    key: str = auto_field()
    value: str = auto_field()


@auto_dataclass
class CarStateSP:
  pass
