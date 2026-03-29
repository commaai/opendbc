""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.gwm.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.GWM_HAVAL_H6: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x873612100XEC56000\xf1\x89S013A01XKN17002',
    ],
  },
}
