""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.byd.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.BYD_ATTO3: {
    (Ecu.hvac, 0x7b3, None): [
      b'\xf1\x8b\x00\x00\x00\xff',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'H7\x00\x11V\xfd\x00\x12!',
    ],
  },
}
