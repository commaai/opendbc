from opendbc.car.structs import CarParams
from opendbc.car.perodua.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.MYVI_PSD: {
    (Ecu.engine, 0x7e0, None): [
      b'\x0391P04220\x00\x00\x00\x00\x00\x00\x00\x00W1001000\x00\x00\x00\x00\x00\x00\x00\x00D39HC23\x00\x00\x00\x00',
    ]
  },
}
