import os
import capnp
from opendbc.car.common.basedir import BASEDIR

# TODO: remove car from cereal/__init__.py and always import from opendbc
capnp.remove_import_hook()
# try:
#   from openpilot_logging.cereal import car
#   # car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])
# except ImportError:
car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR, os.path.join(BASEDIR, "../../openpilot_logging/cereal")])

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule
