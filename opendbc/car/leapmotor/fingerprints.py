from opendbc.car.leapmotor.values import CAR

# CAN fingerprint of CAN1 (bus 0). Besides the periodic messages it has:
#   805, 808 (0x325, 0x328)  sent by an ECU for a few seconds after openpilot's VIN and FW queries, which run first
#   1851, 1883, 1928 (0x73B, 0x75B, 0x788)  diagnostic responses on CAN1 in the first ~75 s after the car is turned on
FINGERPRINTS = {
  CAR.LEAPMOTOR_T03: [{
    234: 8, 235: 8, 237: 8, 257: 8, 258: 8, 290: 8, 291: 8, 292: 8, 293: 8, 294: 8, 304: 8, 417: 8, 418: 8, 420: 8, 421: 8,
    640: 8, 784: 8, 804: 8, 805: 8, 808: 8, 849: 8, 865: 8, 869: 8, 896: 8, 905: 8, 906: 8, 907: 8, 928: 8, 944: 8, 952: 8,
    953: 8, 971: 8, 976: 8, 992: 8, 993: 8, 1217: 8, 1218: 8, 1219: 8, 1424: 8, 1457: 8, 1458: 8, 1459: 8, 1460: 8, 1461: 8,
    1472: 8, 1527: 8, 1583: 8, 1584: 8, 1585: 8, 1851: 8, 1883: 8, 1928: 8
  }],
}

FW_VERSIONS: dict = {}
