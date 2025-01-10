from opendbc.car.structs import CarParams
from opendbc.car.rivian.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.RIVIAN_R1S: {
    (Ecu.eps, 0x730, None): [
    ],
    (Ecu.engine, 0x606, None): [
    ],
  },
}