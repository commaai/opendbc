from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR

Ecu = CarParams.Ecu


FW_VERSIONS_EXT = {
  CAR.KIA_NIRO_EV: {
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00DE  MDPS C 1.00 1.05 56310Q4200\x00 4DEEC105',
    ],
  },
}
