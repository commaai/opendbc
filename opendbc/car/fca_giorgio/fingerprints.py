from opendbc.car.structs import CarParams
from opendbc.car.fca_giorgio.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.ALFA_ROMEO_STELVIO_1ST_GEN: {
    (Ecu.engine, 0x7e0, None): [
      b'PLACEHOLDER',
    ],
  },
}
