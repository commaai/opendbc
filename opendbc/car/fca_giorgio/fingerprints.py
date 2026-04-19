from opendbc.car.structs import CarParams
from opendbc.car.fca_giorgio.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.ALFA_ROMEO_STELVIO_1ST_GEN: {
    (Ecu.engine, 0x7e0, None): [
      b'PLACEHOLDER',
    ],
  },
  CAR.RAM_PROMASTER: {
    (Ecu.eps, 0x18da30f1, None): [
      b'\x01\x01FI06FD00-0\x00\x0b',  # 2025 Promaster 2500 ACC-only
      b'\x01\x01S2FI04EB00\x00\x0b',  # 2022 Promaster 3500 w/LKAS
    ],
  },
}
