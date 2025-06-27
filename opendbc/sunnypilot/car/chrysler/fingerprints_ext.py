from opendbc.car.structs import CarParams
from opendbc.car.chrysler.values import CAR

Ecu = CarParams.Ecu


FW_VERSIONS_EXT = {
  CAR.RAM_1500_5TH_GEN: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68453485AC',
      b'68510283AH',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68552791AA',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'05149390AA ',
      b'68378696AI ',
      b'68500631AF',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'68360085AF',
      b'68360086AL',
      b'68502996AC',
    ],
  },
}
