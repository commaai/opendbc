from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS_EXT = {
  CAR.TESLA_MODEL_3: {
    (Ecu.eps, 0x730, None): [
      b'TeMYG4_Main_0.0.0 (67),E4HP015.02.1',
    ],
  },
}
