from cereal import car
from opendbc.car.volvo.values import CAR

Ecu = car.CarParams.Ecu

FINGERPRINTS = {
  CAR.VOLVO_V60: [
    {
      16: 8, 32: 8, 81: 8, 99: 8, 104: 8, 112: 8,
      277: 8, 307: 8, 320: 8, 328: 8, 336: 8, 343: 8,
      352: 8, 359: 8, 384: 8, 465: 8, 511: 8, 522: 8,
      544: 8, 565: 8, 582: 8, 608: 8, 609: 8, 610: 8,
      624: 8, 648: 8, 665: 8, 673: 8, 704: 8, 706: 8,
      750: 8, 751: 8, 788: 8, 797: 8, 802: 8, 807: 8,
      821: 8, 978: 8,
    },
  ],
}

FW_VERSIONS = {
  CAR.VOLVO_V60: {
    (Ecu.engine, 0x7e0, None): [
      b'31361041 AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  }
}
