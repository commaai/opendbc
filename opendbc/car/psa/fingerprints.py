# ruff: noqa: E501
from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FINGERPRINTS = {
  CAR.PSA_OPEL_CORSA_F: [{
  }],
}

FW_VERSIONS = {
  CAR.PSA_OPEL_CORSA_F: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'212053276',
        b'\xff\xff\x00\x00\x0f\xe8\x18\x05!@Y\xa4\x03\xff\xff\xff\x00\x02\x00\x00\x01\x94\x80\x97'
    ],
    # DIRECTN - Electronic Power Steering
    (Ecu.eps, 0x6B5, None): [
        b'6077GC0817309',
        b'\xbfP\x00\x00\x13j\x07\x06\x15\xb5@\xf5\x03\xff\xff\xff\x00\x02\x00\x00\x01\x944g'
    ],
    # HCU2 - Hybrid Control Unit
    (Ecu.hybrid, 0x6A6, None): [
        b'210306062100',
        b'\xff\xff\x00\x00\r\n\x06\x03!\x03\x01\x12\x01\xff\xff\xff\x00\x02\x00\x00\x02\x94\x86b'
    ],
    # MSB - Electronic Brake Booster
    (Ecu.electricBrakeBooster, 0x6B4, None): [
        b'521021900860',
        b'\xff\xff\x00\x00t\x01\x11\x01!\x01\x040\x15\xff\xff\xff\x00\x02\x00\x00\xfe\x95\x08w'
    ],
    # VCU - Vehicle Control Unit
    (Ecu.engine, 0x6A2, None): [
        b'9210126909',
        b'\xf2i\x00\x00\r\x99\x11\x05\x15\x01!\xb2\x01!\x07#\xfd\xd4S\xe2\x02\x96E '
    ],
    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        b'085095700857210527',
        b'\x00\x00\x00\x00\x03\x93!\x08 \x01\xc2\x12\x12\xff\xff\xff\x00\x02\x00\x00\x01\x94'
    ],
  }
}


