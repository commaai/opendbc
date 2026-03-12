import os
from typing import Any
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
CarStateT: type = Any  # pyrefly: ignore[bad-assignment]
RadarDataT: type = Any  # pyrefly: ignore[bad-assignment]
CarControlT: type = Any  # pyrefly: ignore[bad-assignment]
CarParamsT: type = Any  # pyrefly: ignore[bad-assignment]
