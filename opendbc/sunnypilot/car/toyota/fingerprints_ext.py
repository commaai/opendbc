from opendbc.car.structs import CarParams
from opendbc.car.toyota.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS_EXT = {
  CAR.TOYOTA_WILDLANDER: {
    (Ecu.engine, 0x700, None): [
      b'\x01896630R57001\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x7b0, None): [
      b'\x01F152642C4000\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x7a1, None): [
      b'\x018965B4221000\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x750, 0x6d): [
      b'\x028646F0R01000\x00\x00\x00\x008646G4202000\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x750, 0xf): [
      b'\x018821F3301400\x00\x00\x00\x00',
    ],
  },
}
