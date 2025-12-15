""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        b'085095700857210527', # Peugeot 208 ACC
        b'285160381930060025B',# Peugeot 2008 ACC
    ],
  },
  CAR.PSA_PEUGEOT_508: {
    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        b'085095308910190312', # Citroen C5 Aircross CC-only
        b'085065315928191130', # Peugeot 508 Hybrid, Citroen Berlingo VP K9
    ],
  },
}
