""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.leapmotor.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.LEAPMOTOR_B10: {
    (Ecu.eps, 0x735, None): [
      b'3401100-AG01\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x736, None): [
      b'3565101-AG03\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x756, None): [
      b'8021010-AH01\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}
