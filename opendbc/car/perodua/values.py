# flake8: noqa
from opendbc.car.structs import CarParams
from opendbc.car import dbc_dict

from collections import defaultdict
Ecu = CarParams.Ecu

HUD_MULTIPLIER = 1.04

class CAR:
  MYVI_PSD = "PERODUA MYVI PSD"

FINGERPRINTS = {
  CAR.MYVI_PSD: [{
    160: 5, 161: 8, 164: 8, 165: 4, 308: 6, 385: 3, 398: 8, 399: 8, 400: 8, 405: 5, 409: 8, 410: 8, 416: 8, 417: 7, 427: 8, 429: 8, 448: 4, 449: 8, 464: 8, 496: 5, 516: 8, 520: 6, 524: 6, 608: 8, 609: 8, 624: 8, 625: 8, 627: 8, 628: 8, 682: 2, 752: 8, 834: 8, 848: 5, 856: 8, 857: 4, 900: 4, 976: 5, 980: 8, 1012: 7, 1032: 8, 1033: 8, 1034: 8, 1088: 8, 1090: 8, 1100: 8, 1152: 8, 1160: 4, 1162: 8, 1163: 8, 1164: 8, 1168: 8, 1176: 3, 1188: 8, 1200: 3, 1204: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1248: 8, 1267: 8, 1312: 8, 1329: 8, 1408: 8, 1409: 8, 1410: 8, 1416: 8, 1417: 8, 1418: 8, 1434: 8, 1435: 8, 1542: 2, 1798: 3
  }]
}

ECU_FINGERPRINT = {
  # ASA Camera CAN fingerprint
  Ecu.fwdCamera: [679, 680, 681, 1267]
}

DBC = {
  CAR.MYVI_PSD: dbc_dict('perodua_psd_pt', None),
}


BRAKE_SCALE = defaultdict(lambda: 1, {CAR.MYVI_PSD: 3.3})
GAS_SCALE = defaultdict(lambda: 2600, {CAR.MYVI_PSD: 0.35})

ACC_CAR = {CAR.MYVI_PSD}
