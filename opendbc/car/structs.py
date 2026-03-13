import os
from typing import Any, TypeAlias
import capnp
from opendbc.car.common.basedir import BASEDIR

# TODO: remove car from cereal/__init__.py and always import from opendbc
try:
  from cereal import car
except ImportError:
  capnp.remove_import_hook()  # pyrefly: ignore[missing-attribute] - capnp dynamically exposes this
  car = capnp.load(os.path.join(BASEDIR, "car.capnp"))  # pyrefly: ignore[missing-attribute] - capnp dynamically exposes this

CarState: Any = car.CarState
RadarData: Any = car.RadarData
CarControl: Any = car.CarControl
CarParams: Any = car.CarParams

# capnp struct module types, accessed via capnp.lib.capnp._StructModule
CarStateT: TypeAlias = Any
RadarDataT: TypeAlias = Any
CarControlT: TypeAlias = Any
CarParamsT: TypeAlias = Any
