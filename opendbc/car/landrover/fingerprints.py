from opendbc.car.structs import CarParams
from opendbc.car.landrover.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.LANDROVER_DEFENDER_2023: {
    (Ecu.eps, 0x730, None): [
      b'M8B2-14C217-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.shiftByWire, 0x732, None): [
      b'N8B2-14C561-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'M8B2-14D049-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'P8B2-12K532-LEB\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },

  # TODO
  CAR.RANGEROVER_VOGUE_2017: {
    (Ecu.eps, 0x730, None): [
      b'M8B2-14C217-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.shiftByWire, 0x732, None): [
      b'N8B2-14C561-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'M8B2-14D049-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'P8B2-12K532-LE\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}
