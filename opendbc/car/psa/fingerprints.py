""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    (Ecu.abs, 0x6ad, None): [
      b'085095700857210527',
      b'085095706198220818',
      b'085095700473220908',
      b'085135705863191103',
      b'285160381930060025B',
    ],
  },
  CAR.PSA_PEUGEOT_508: {
    (Ecu.abs, 0x6ad, None): [
      b'085065315928191130',
      b'085095308910190312',
      b'085095701207240115',
      b'085135702688200218',
    ],
  },
}
