import os
import capnp
from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()  # ty: ignore[unresolved-attribute]
car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])  # ty: ignore[unresolved-attribute]

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule  # ty: ignore[unresolved-attribute]
RadarDataT = capnp.lib.capnp._StructModule  # ty: ignore[unresolved-attribute]
CarControlT = capnp.lib.capnp._StructModule  # ty: ignore[unresolved-attribute]
CarParamsT = capnp.lib.capnp._StructModule  # ty: ignore[unresolved-attribute]
