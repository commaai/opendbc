from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR

Ecu = CarParams.Ecu


FW_VERSIONS_EXT = {
  CAR.KIA_NIRO_EV: {
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00DE  MDPS C 1.00 1.05 56310Q4200\x00 4DEEC105',
    ],
  },
  CAR.KIA_CEED_PHEV_2022_NON_SCC: {
    (Ecu.eps, 0x7D4, None): [
      b'\xf1\x00CD  MDPS C 1.00 1.01 56310-XX000 4CPHC101',
    ],
    (Ecu.fwdCamera, 0x7C4, None): [
      b'\xf1\x00CDH LKAS AT EUR LHD 1.00 1.01 99211-CR700 931',
    ],
  },
}
