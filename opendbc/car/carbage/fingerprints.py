from opendbc.car.structs import CarParams
from opendbc.car.carbage.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.CARBAGE: {
    (Ecu.engine, 0x720, None): [
      b'carbage',
    ],
  },
}
