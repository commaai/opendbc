from opendbc.car.renault.values import CAR
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

FINGERPRINTS = {}

FW_VERSIONS = {
  CAR.RENAULT_5_ETECH: {
    (Ecu.fwdCamera, 0x1bde62f2, None): [
      b'284A61243R',
    ],
  },
}
