from opendbc.car.structs import CarParams
from opendbc.car.mg.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.MG_5_EV: {
    (Ecu.eps, 0x730, None): [
      b'R1TS_v3.4.1(51),3.4.1\x00',
    ],
  },
}
