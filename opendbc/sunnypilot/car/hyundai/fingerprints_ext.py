from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR

Ecu = CarParams.Ecu


FW_VERSIONS_EXT = {
  CAR.KIA_NIRO_EV: {
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00DE  MDPS C 1.00 1.05 56310Q4200\x00 4DEEC105',
    ],
  },
  CAR.KIA_CEED_PHEV_2022_NON_SCC: {
    (Ecu.eps, 0x7D4, None): [
      b'\xf1\x00CD  MDPS C 1.00 1.01 56310-XX000 4CPHC101',
    ],
    (Ecu.fwdCamera, 0x7C4, None): [
      b'\xf1\x00CDH LKAS AT EUR LHD 1.00 1.01 99211-CR700 931',
    ],
  },
  # TODO-SP: HYUNDAI_KONA_EV_NON_SCC has the same FW versions as HYUNDAI_KONA_EV, in the future we may
  #          allow similar FW versions across different platforms
  # CAR.HYUNDAI_KONA_EV_NON_SCC: {
  #   (Ecu.abs, 0x7d1, None): [
  #     b'\xf1\x00OS IEB \x02 212 \x11\x13 58520-K4000',
  #   ],
  #   (Ecu.eps, 0x7d4, None): [
  #     b'\xf1\x00OS  MDPS C 1.00 1.04 56310K4000\x00 4OEDC104',
  #   ],
  #   (Ecu.fwdCamera, 0x7c4, None): [
  #     b'\xf1\x00OSE LKAS AT USA LHD 1.00 1.00 95740-K4100 W40',
  #   ],
  # },
}
