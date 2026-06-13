""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.byd.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.BYD_ATTO_3: {
    (Ecu.engine, 0x7e0, None): [
      b'PLACEHOLDER_FOR_VIN_FINGERPRINT',
    ],
  },
}
