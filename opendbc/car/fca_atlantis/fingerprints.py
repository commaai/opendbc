from opendbc.car.structs import CarParams
from opendbc.car.fw_query_definitions import OfflineFwVersions
from opendbc.car.fca_atlantis.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS: OfflineFwVersions = {
  CAR.RAM_PROMASTER: {
    (Ecu.eps, 0x18da30f1, None): [
      b'\x01\x01FI06FF00-0\x00\x0b',  # 2026 Promaster 2500 ACC-only
      b'\x01\x01FI06FD00-0\x00\x0b',  # 2025 Promaster 2500 ACC-only
      b'\x01\x01S2FI04EB00\x00\x0b',  # 2022 Promaster 3500 w/LKAS
    ],
    (Ecu.abs, 0x18da28f1, None): [
      b'\x01\x01\x18\x07\x00\x00      \x00\x01',  # 2025-26 Promaster 2500 ACC-only
      b'\x01\x01\x18\x06\x01\x00      \x00\x01',  # 2022 Promaster 3500 w/LKAS (2 vans)
    ],
    (Ecu.fwdCamera, 0x18da31f1, None): [
      b'\x01\x01\x06\x00\x06 \x00\x03 \x00\x00\x01\x00&',  # 2022 Promaster 3500 w/LKAS (2 vans), 2025 Promaster 2500 ACC-only
    ],
  },
}
