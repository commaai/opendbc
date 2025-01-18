from opendbc.car.structs import CarParams
from opendbc.car.perodua.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.MYVI_PSD: {
    (Ecu.abs, 0x7b0, None): [
      b'D51A10101BOSCH\x00\x00',
    ],
    (Ecu.dsu, 0x791, None): [
      b'3920\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    ],
    (Ecu.engine, 0x7e0, None): [
      b'\x0391P04220\x00\x00\x00\x00\x00\x00\x00\x00W1001000\x00\x00\x00\x00\x00\x00\x00\x00D39HC23\x00\x00\x00\x00',
      b'\x0391P04220\x00\x00\x00\x00\x00\x00\x00\x00W1001000\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x00\x00\x00\x00\x00',
    ],
    (Ecu.srs, 0x780, None): [
      b'D20Nfr3sMY000000',
    ],
  },
}
