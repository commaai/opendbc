from opendbc.car.structs import CarParams
from opendbc.car.byd.values import CAR

Ecu = CarParams.Ecu

# Added fingerprint for BYD ATTO3 India Version. Line 2 is the BYD Fingerprint.

FINGERPRINTS = {
    CAR.BYD_ATTO3: [{
        85: 8, 140: 8, 204: 20, 213: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 307: 8, 309: 8,
        324: 8, 327: 8, 337: 8, 356: 8, 371: 8, 384: 8, 385: 3, 410: 8, 418: 8, 450: 8, 482: 8,
        496: 8, 508: 8, 522: 8, 536: 8, 537: 8, 544: 8, 546: 8, 547: 8, 576: 8, 577: 8, 578: 8,
        588: 8, 629: 8, 638: 8, 639: 8, 660: 8, 665: 8, 682: 2, 692: 8, 694: 8, 724: 8, 748: 8,
        786: 8, 790: 8, 792: 8, 797: 8, 798: 8, 800: 8, 801: 8, 802: 8, 803: 8, 812: 8, 813: 8,
        814: 8, 815: 8, 827: 8, 828: 8, 829: 8, 833: 8, 834: 8, 835: 8, 836: 8, 843: 8, 847: 8,
        848: 8, 854: 8, 860: 8, 863: 8, 879: 8, 884: 8, 906: 8, 944: 8, 951: 8, 965: 8, 973: 8,
        985: 8, 1004: 8, 1020: 8, 1023: 8, 1028: 8, 1031: 8, 1036: 8, 1037: 8, 1040: 8, 1048: 8,
        1052: 8, 1058: 8, 1074: 8, 1076: 8, 1098: 8, 1107: 8, 1141: 8, 1168: 8, 1178: 8, 1184: 8,
        1189: 8, 1192: 8, 1193: 8, 1211: 8, 1215: 8, 1217: 8, 1246: 8, 1269: 8, 1274: 8, 1278: 8,
        1297: 8, 1298: 8, 1319: 8, 1322: 8, 1542: 2, 1590: 8, 1687: 8, 1792: 8, 1798: 3, 1799: 8,
        1810: 8, 1813: 8, 1824: 8, 1832: 8, 1840: 8, 1856: 8, 1858: 8, 1859: 8, 1862: 8, 1872: 8,
        1879: 8, 1888: 8, 1892: 8, 1927: 8, 1937: 8, 1953: 8, 1968: 8, 1988: 8, 2000: 8, 2001: 8,
        2004: 8, 2012: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8, 2027: 8
    },
        {
        85: 8, 140: 8, 213: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 307: 8, 309: 8, 324: 8,
        327: 8, 330: 8, 337: 8, 356: 8, 371: 8, 384: 8, 410: 8, 418: 8, 450: 8, 482: 8, 496: 8,
        508: 8, 522: 8, 536: 8, 537: 8, 544: 8, 546: 8, 547: 8, 576: 8, 577: 8, 578: 8, 587: 8,
        588: 8, 629: 8, 638: 8, 639: 8, 660: 8, 665: 8, 692: 8, 694: 8, 724: 8, 748: 8, 786: 8,
        790: 8, 792: 8, 797: 8, 798: 8, 800: 8, 801: 8, 802: 8, 803: 8, 812: 8, 813: 8, 814: 8,
        815: 8, 831: 8, 833: 8, 834: 8, 835: 8, 836: 8, 843: 8, 847: 8, 848: 8, 854: 8, 860: 8,
        863: 8, 879: 8, 884: 8, 906: 8, 944: 8, 951: 8, 965: 8, 973: 8, 985: 8, 1004: 8, 1020: 8,
        1023: 8, 1028: 8, 1031: 8, 1037: 8, 1040: 8, 1048: 8, 1052: 8, 1058: 8, 1074: 8, 1076: 8,
        1098: 8, 1107: 8, 1141: 8, 1178: 8, 1184: 8, 1189: 8, 1193: 8, 1211: 8, 1215: 8, 1217: 8,
        1246: 8, 1252: 8, 1274: 8, 1278: 8, 1297: 8, 1298: 8, 1319: 8, 1322: 8, 1792: 8, 1813: 8,
        1817: 8, 1840: 8, 1842: 8, 1856: 8, 1860: 8, 1871: 8, 1872: 8, 1943: 8, 1948: 8, 1969: 8,
        1971: 8, 1975: 8, 1979: 8, 1988: 8, 1990: 8, 2000: 8, 2001: 8, 2004: 8, 2017: 8, 2018: 8,
        2027: 8
    },
        {
        85: 8, 140: 8, 213: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 307: 8, 309: 8, 324: 8,
        327: 8, 330: 8, 337: 8, 356: 8, 371: 8, 384: 8, 385: 3, 410: 8, 418: 8, 432: 8, 450: 8,
        482: 8, 496: 8, 508: 8, 522: 8, 536: 8, 537:      8, 544: 8, 546: 8, 547: 8, 576: 8, 577: 8,
        578: 8, 588: 8, 629: 8, 638: 8, 639: 8, 660: 8, 665: 8, 682: 2, 692: 8, 694: 8, 724: 8, 748: 8,
        786: 8, 790: 8, 792: 8, 797: 8, 798: 8, 800: 8, 801: 8, 802: 8, 803: 8, 812: 8, 813: 8, 814:
        8, 815: 8, 831: 8, 833: 8, 834: 8, 835: 8, 836: 8, 843: 8, 847: 8, 848: 8, 854: 8, 860: 8,
        863: 8, 879: 8, 884: 8, 906: 8, 944: 8, 951: 8, 965: 8, 973: 8, 985: 8, 1004: 8, 1020: 8,
        1023: 8, 1028: 8, 1031: 8, 1036: 8, 1037: 8, 1040:      8, 1048: 8, 1052: 8, 1058: 8,
        1074: 8, 1076: 8, 1098: 8, 1107: 8, 1141: 8, 1178: 8, 1184: 8, 1189: 8, 1193: 8, 1211: 8,
        1215: 8, 1217: 8, 1246: 8, 1274: 8, 1278: 8, 1297: 8, 1319: 8, 1322: 8, 1542: 2, 1798: 3,
        1824: 8, 1832: 8
    }],
    CAR.HAN_DM_20: [{
        85: 8, 140: 8, 213: 8, 269: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 303: 8, 307: 8,
        309: 8, 315: 8, 384: 8, 496: 8, 530: 8, 536: 8, 544: 8, 546: 8, 547: 8, 576: 8, 578: 8,
        588: 8, 660: 8, 694: 8, 790: 8, 792: 8, 797: 8, 798: 8, 801: 8, 802: 8, 813: 8, 814: 8,
        815: 8, 833: 8, 834: 8, 836: 8, 843: 8, 860: 8, 884: 8, 916: 8, 918: 8, 926: 8, 940: 8,
        941: 8, 944: 8, 948: 8, 985: 8, 988: 8, 1004: 8, 1020: 8, 1037: 8, 1040: 8, 1058: 8,
        1074: 8, 1141: 8, 1172: 8, 1178: 8, 1180: 8, 1193: 8, 1246: 8, 1293: 8, 1793: 8, 1796: 8,
        1804: 8, 1904: 8, 1905: 8, 1912: 8, 1913: 8, 1986: 8, 2004: 8, 2034: 8, 2042: 8
    }],
    CAR.HAN_EV_20: [{
        85: 8, 140: 8, 213: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 303: 8, 307: 8, 308: 8,
        309: 8, 315: 8, 464: 8, 465: 8, 480: 8, 496: 8, 536: 8, 544: 8, 546: 8, 547: 8, 576: 8,
        578: 8, 588: 8, 660: 8, 694: 8, 790: 8, 792: 8, 797: 8, 798: 8, 801: 8, 802: 8, 812: 8,
        813: 8, 814: 8, 815: 8, 833: 8, 834: 8, 836: 8, 843: 8, 860: 8, 863: 8, 879: 8, 884: 8,
        916: 8, 918: 8, 920: 8, 926: 8, 940: 8, 941: 8, 944: 8, 948: 8, 965: 8, 976: 8, 985: 8,
        988: 8, 1004: 8, 1020: 8, 1036: 8, 1037: 8, 1040: 8, 1048: 8, 1058: 8, 1074: 8, 1141: 8,
        1172: 8, 1178: 8, 1180: 8, 1193: 8, 1246: 8, 1268: 8, 1793: 8, 1794: 8, 1797: 8,
        1798: 8, 1801: 8, 1808: 8, 1809: 8, 1811: 8, 1812: 8, 1824: 8, 1827: 8, 1828: 8, 1829: 8,
        1830: 8, 1842: 8, 1843: 8, 1845: 8, 1847: 8, 1858: 8, 1859: 8, 1862: 8, 1863: 8, 1872: 8,
        1873: 8, 1874: 8, 1876: 8, 1890: 8, 1891: 8, 1894: 8, 1904: 8, 1905: 8, 1912: 8, 1913: 8,
        1920: 8, 1921: 8, 1922: 8, 1923: 8, 1925: 8, 1927: 8, 1939: 8, 1940: 8, 1943: 8, 1959: 8,
        1971: 8, 1973: 8, 1984: 8, 1986: 8, 1987: 8, 1991: 8, 1994: 8, 2002: 8, 2004: 8, 2006: 8,
        2012: 8, 2033: 8, 2034: 8, 2042: 8
    }],
}

# Found this snippet in car/gm/fingerprint.py codes. This takes care of FW_VERSIONS
FW_VERSIONS: dict[str, dict[tuple, list[bytes]]] = {
}
