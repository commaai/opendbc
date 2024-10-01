import os
import capnp

OPENDBC_CAR_PATH = os.path.dirname(os.path.abspath(__file__))

try:
  from cereal import car
except ImportError:
  capnp.remove_import_hook()
  car = capnp.load(os.path.join(OPENDBC_CAR_PATH, "car.capnp"))

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams
