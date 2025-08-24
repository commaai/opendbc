from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR

Ecu = CarParams.Ecu


FW_VERSIONS_EXT = {
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
  CAR.GENESIS_G70_2021_NON_SCC: {
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00IK  MDPS R 1.00 1.08 57700-G9200 4I2CL108',
    ],
    (Ecu.fwdRadar, 0x7d0, None): [
      b'\xf1\x00IK__ SCC --CUP      1.00 1.02 96400-G9100         ',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00IK  MFC  MT USA LHD 1.00 1.01 95740-G9000 170920',
    ],
  },
  CAR.HYUNDAI_KONA_NON_SCC: {
    # (Ecu.abs, 0x7d1, None): [
    #   b'\xf1\x816V5RAJ00040.ELF\xf1\x00\x00\x00\x00\x00\x00\x00',
    # ],
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00OS  MDPS C 1.00 1.05 56310J9030\x00 4OSDC105',
      b'\xf1\x00OS  MDPS C 1.00 1.04 56310J9030\x00 4OSDC104',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00OS9 LKAS AT USA LHD 1.00 1.00 95740-J9200 g30',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x006T6J0_C2\x00\x006T6K1051\x00\x00TOS4N20NS2\x00\x00\x00\x00',
    ],
  },
  CAR.KIA_FORTE_2019_NON_SCC: {
    (Ecu.eps, 0x7D4, None): [
      b'\xf1\x00BD  MDPS C 1.00 1.04 56310/M6000 4BDDC104',
    ],
    (Ecu.fwdCamera, 0x7C4, None): [
      b'\xf1\x00BD  LKAS AT USA LHD 1.00 1.02 95740-M6000 J31',
    ],
    # (Ecu.abs, 0x7d1, None): [
    #   b'\xf1\x816VFRAF00018.ELF\xf1\x00\x00\x00\x00\x00\x00\x00',
    # ],
    # (Ecu.transmission, 0x7e1, None): [
    #   b'\xf1\x87CXJQAM4966515JB0x\xa9\x98\x9b\x99fff\x98feg\x88\x88w\x88Ff\x8f\xff{\xff\xff\xff\xa8\xf6\xf1\x816V2C1051\x00\x00\xf1\x006V2B0_C2\x00\x006V2C1051\x00\x00CBD0N20NS8q\xc1&\xd2',  # noqa: E501
    # ],
  },
  CAR.KIA_FORTE_2021_NON_SCC: {
    (Ecu.eps, 0x7D4, None): [
      b'\xf1\x00BD  MDPS C 1.00 1.08 56310M6000\x00 4BDDC108',
    ],
    (Ecu.fwdCamera, 0x7C4, None): [
      b'\xf1\x00BD  LKAS AT USA LHD 1.00 1.04 95740-M6000 J33',
    ],
    # (Ecu.abs, 0x7d1, None): [
    #   b'\xf1\x816VFRAL00010.ELF\xf1\x00\x00\x00\x00\x00\x00\x00',
    # ],
    # (Ecu.transmission, 0x7e1, None): [
    #   b'\xf1\x87CXLQAM0906975JB0\x89\x88\xa6\x8aVfug\xba\x87\x94yffuxgfo\xff\x8b\xff\xff\xff\x91\x82\xf1\x816V2C1051\x00\x00\xf1\x006V2B0_C2\x00\x006V2C1051\x00\x00CBD0N20NS8q\xc1&\xd2',  # noqa: E501
    # ],
  },
  CAR.KIA_SELTOS_2023_NON_SCC: {
    (Ecu.abs, 0x7d1, None): [
      b'\xf1\x00SP ESC \t 101"\t\x01 58910-Q5510',
      b'\xf1\x00SP ESC \r 100"\x04\x01 58910-Q5510',
    ],
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00SP2 MDPS C 1.00 1.04 56310Q5240  4SPSC104',
      b'\xf1\x00SP2 MDPS C 1.00 1.01 56300Q5920          ',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00SP2 MFC  AT USA LHD 1.00 1.03 99210-Q5500 230208',
      b'\xf1\x00SP2 MFC  AT AUS RHD 1.00 1.02 99210-Q5500 220624',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x006V2B0_C2\x00\x006V2D5051\x00\x00CSP2N20NL0\x00\x00\x00\x00',
      b'\xf1\x006V2B0_C2\x00\x006V2D4051\x00\x00CSP2N20KL1\x00\x00\x00\x00',
    ],
  },
  CAR.HYUNDAI_ELANTRA_2022_NON_SCC: {
    (Ecu.eps, 0x7d4, None): [
      # b'\xf1\x8756310AA030\x00\xf1\x00CN7 MDPS C 1.00 1.06 56310AA030\x00 4CNDC106',
      b'\xf1\x00CN7 MDPS R 1.00 1.04 57700-IB000 4CNNP104',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00CN7 MFC  AT USA LHD 1.00 1.01 99210-AB000 210205',
      b'\xf1\x00CN7 MFC  AT USA LHD 1.00 1.00 99210-IB000 210531',
    ],
    (Ecu.abs, 0x7d1, None): [
      # b'\xf1\x8758910-AB500\xf1\x00CN ESC \t 100 \x06\x01 58910-AB500',
      b'\xf1\x00CN ESC \t 100!\x05\x01 58910-IB000',
    ],
    (Ecu.transmission, 0x7e1, None): [
      # b'\xf1\x87CXNQEM4091445JB3g\x98\x98\x89\x99\x87gv\x89wuwgwv\x89hD_\xffx\xff\xff\xff\x86\xeb\xf1\x89HT6VA640A1\xf1\x82CCN0N20NS5\x00\x00\x00\x00\x00\x00',  # noqa: E501
      b'\xf1\x00T02601BL  T02900A1  WCN7T20XXX900NS4\xf7\xccz\xf6',
    ],
  },
  CAR.HYUNDAI_BAYON_1ST_GEN_NON_SCC: {
    # TODO: Check working route for more FW
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00BC3 LKA  AT EUR LHD 1.00 1.01 99211-Q0100 261'
    ],
  },
}
