import os
import capnp

OPENDBC_CAR_PATH = os.path.dirname(os.path.abspath(__file__))
capnp.remove_import_hook()

car = capnp.load(os.path.join(OPENDBC_CAR_PATH, "car.capnp"))
