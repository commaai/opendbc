from opendbc.car.structs import CarParams
from opendbc.car.subaru.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.SUBARU_ASCENT: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa5 \x19\x02\x00',
      b'\xa5 !\x02\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'\x05\xc0\xd0\x00',
      b'\x85\xc0\xd0\x00',
      b'\x95\xc0\xd0\x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00d\xb9\x00\x00\x00\x00',
      b'\x00\x00d\xb9\x1f@ \x10',
      b'\x00\x00e@\x00\x00\x00\x00',
      b'\x00\x00e@\x1f@ $',
      b"\x00\x00e~\x1f@ '",
    ],
  },
  CAR.SUBARU_ASCENT_2023: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa5 #\x03\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'%\xc0\xd0\x11',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x05!\x08\x1dK\x05!\x08\x01/',
    ],
  },
  CAR.SUBARU_LEGACY: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa1  \x02\x01',
      b'\xa1  \x02\x02',
      b'\xa1  \x03\x03',
      b'\xa1  \x04\x01',
    ],
    (Ecu.eps, 0x746, None): [
      b'\x9b\xc0\x11\x00',
      b'\x9b\xc0\x11\x02',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00e\x80\x00\x1f@ \x19\x00',
      b'\x00\x00e\x9a\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.SUBARU_IMPREZA: {
    (Ecu.abs, 0x7b0, None): [
      b'z\x84\x19\x90\x00',
      b'z\x94\x08\x90\x00',
      b'z\x94\x08\x90\x01',
      b'z\x94\x0c\x90\x00',
      b'z\x94\x0c\x90\x01',
      b'z\x94.\x90\x00',
      b'z\x94?\x90\x00',
      b'z\x9c\x19\x80\x01',
      b'\xa2 \x185\x00',
      b'\xa2 \x193\x00',
      b'\xa2 \x194\x00',
      b'\xa2 \x19`\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'z\xc0\x00\x00',
      b'z\xc0\x04\x00',
      b'z\xc0\x08\x00',
      b'z\xc0\n\x00',
      b'z\xc0\x0c\x00',
      b'\x8a\xc0\x00\x00',
      b'\x8a\xc0\x10\x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00c\xf4\x00\x00\x00\x00',
      b'\x00\x00c\xf4\x1f@ \x07',
      b'\x00\x00d)\x00\x00\x00\x00',
      b'\x00\x00d)\x1f@ \x07',
      b'\x00\x00dd\x00\x00\x00\x00',
      b'\x00\x00dd\x1f@ \x0e',
      b'\x00\x00d\xb5\x1f@ \x0e',
      b'\x00\x00d\xdc\x00\x00\x00\x00',
      b'\x00\x00d\xdc\x1f@ \x0e',
      b'\x00\x00e\x02\x1f@ \x14',
      b'\x00\x00e\x1c\x00\x00\x00\x00',
      b'\x00\x00e\x1c\x1f@ \x14',
      b'\x00\x00e+\x00\x00\x00\x00',
      b'\x00\x00e+\x1f@ \x14',
    ],
  },
  CAR.SUBARU_IMPREZA_2020: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa2 \x193\x00',
      b'\xa2 \x194\x00',
      b'\xa2  `\x00',
      b'\xa2 !3\x00',
      b'\xa2 !6\x00',
      b'\xa2 !`\x00',
      b'\xa2 !i\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'\n\xc0\x04\x00',
      b'\n\xc0\x04\x01',
      b'\x9a\xc0\x00\x00',
      b'\x9a\xc0\x04\x00',
      b'\x9a\xc0\n\x01',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00eb\x1f@ "',
      b'\x00\x00eq\x00\x00\x00\x00',
      b'\x00\x00eq\x1f@ "',
      b'\x00\x00e\x8f\x00\x00\x00\x00',
      b'\x00\x00e\x8f\x1f@ )',
      b'\x00\x00e\x92\x00\x00\x00\x00',
      b'\x00\x00e\xa4\x00\x00\x00\x00',
      b'\x00\x00e\xa4\x1f@ (',
    ],
  },
  CAR.SUBARU_CROSSTREK_HYBRID: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa2 \x19e\x01',
      b'\xa2 !e\x01',
    ],
    (Ecu.eps, 0x746, None): [
      b'\n\xc2\x01\x00',
      b'\x9a\xc2\x01\x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00el\x1f@ #',
    ],
  },
  CAR.SUBARU_FORESTER: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa3 \x18\x14\x00',
      b'\xa3 \x18&\x00',
      b'\xa3 \x19\x14\x00',
      b'\xa3 \x19&\x00',
      b'\xa3 \x19h\x00',
      b'\xa3  \x14\x00',
      b'\xa3  \x14\x01',
    ],
    (Ecu.eps, 0x746, None): [
      b'\x8d\xc0\x00\x00',
      b'\x8d\xc0\x04\x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00e!\x00\x00\x00\x00',
      b'\x00\x00e!\x1f@ \x11',
      b'\x00\x00e^\x00\x00\x00\x00',
      b'\x00\x00e^\x1f@ !',
      b'\x00\x00e`\x00\x00\x00\x00',
      b'\x00\x00e`\x1f@  ',
      b'\x00\x00e\x97\x00\x00\x00\x00',
      b'\x00\x00e\x97\x1f@ 0',
    ],
  },
  CAR.SUBARU_FORESTER_HYBRID: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa3 \x19T\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'\x8d\xc2\x00\x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00eY\x1f@ !',
    ],
  },
  CAR.SUBARU_FORESTER_PREGLOBAL: {
    (Ecu.abs, 0x7b0, None): [
      b'm\x97\x14@',
      b'}\x97\x14@',
    ],
    (Ecu.eps, 0x746, None): [
      b'm\xc0\x10\x00',
      b'}\xc0\x10\x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00c\xe9\x00\x00\x00\x00',
      b'\x00\x00c\xe9\x1f@ \x03',
      b'\x00\x00d5\x1f@ \t',
      b'\x00\x00d\xd3\x1f@ \t',
    ],
  },
  CAR.SUBARU_LEGACY_PREGLOBAL: {
    (Ecu.abs, 0x7b0, None): [
      b'[\x97D\x00',
      b'[\xba\xc4\x03',
      b'k\x97D\x00',
      b'k\x9aD\x00',
      b'{\x97D\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'K\xb0\x00\x01',
      b'[\xb0\x00\x01',
      b'k\xb0\x00\x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00c\x94\x1f@\x10\x08',
      b'\x00\x00c\xb7\x1f@\x10\x16',
      b'\x00\x00c\xec\x1f@ \x04',
    ],
  },
  CAR.SUBARU_OUTBACK_PREGLOBAL: {
    (Ecu.abs, 0x7b0, None): [
      b'[\xba\xac\x03',
      b'[\xf7\xac\x00',
      b'[\xf7\xac\x03',
      b'[\xf7\xbc\x03',
      b'k\x97\xac\x00',
      b'k\x9a\xac\x00',
      b'{\x97\xac\x00',
      b'{\x9a\xac\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'K\xb0\x00\x00',
      b'K\xb0\x00\x02',
      b'[\xb0\x00\x00',
      b'k\xb0\x00\x00',
      b'{\xb0\x00\x01',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00c\x90\x1f@\x10\x0e',
      b'\x00\x00c\x94\x00\x00\x00\x00',
      b'\x00\x00c\x94\x1f@\x10\x08',
      b'\x00\x00c\xb7\x1f@\x10\x16',
      b'\x00\x00c\xd1\x1f@\x10\x17',
      b'\x00\x00c\xec\x1f@ \x04',
    ],
  },
  CAR.SUBARU_OUTBACK_PREGLOBAL_2018: {
    (Ecu.abs, 0x7b0, None): [
      b'\x8b\x97\xac\x00',
      b'\x8b\x97\xbc\x00',
      b'\x8b\x99\xac\x00',
      b'\x8b\x9a\xac\x00',
      b'\x9b\x97\xac\x00',
      b'\x9b\x97\xbe\x10',
      b'\x9b\x9a\xac\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'{\xb0\x00\x00',
      b'{\xb0\x00\x01',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00df\x1f@ \n',
      b'\x00\x00d\x95\x00\x00\x00\x00',
      b'\x00\x00d\x95\x1f@ \x0f',
      b'\x00\x00d\xfe\x00\x00\x00\x00',
      b'\x00\x00d\xfe\x1f@ \x15',
      b'\x00\x00e\x19\x1f@ \x15',
    ],
  },
  CAR.SUBARU_OUTBACK: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa1  \x06\x00',
      b'\xa1  \x06\x01',
      b'\xa1  \x06\x02',
      b'\xa1  \x07\x00',
      b'\xa1  \x07\x02',
      b'\xa1  \x07\x03',
      b'\xa1  \x08\x00',
      b'\xa1  \x08\x01',
      b'\xa1  \x08\x02',
      b'\xa1 "\t\x00',
      b'\xa1 "\t\x01',
    ],
    (Ecu.eps, 0x746, None): [
      b'\x1b\xc0\x10\x00',
      b'\x9b\xc0\x10\x00',
      b'\x9b\xc0\x10\x02',
      b'\x9b\xc0 \x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x00\x00eJ\x00\x00\x00\x00\x00\x00',
      b'\x00\x00eJ\x00\x1f@ \x19\x00',
      b'\x00\x00e\x80\x00\x1f@ \x19\x00',
      b'\x00\x00e\x9a\x00\x00\x00\x00\x00\x00',
      b'\x00\x00e\x9a\x00\x1f@ 1\x00',
    ],
  },
  CAR.SUBARU_FORESTER_2022: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa3 !v\x00',
      b'\xa3 !x\x00',
      b'\xa3 "v\x00',
      b'\xa3 "x\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'-\xc0\x040',
      b'-\xc0%0',
      b'=\xc0%\x02',
      b'=\xc04\x02',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\x04!\x01\x1eD\x07!\x00\x04,',
      b'\x04!\x08\x01.\x07!\x08\x022',
      b'\r!\x08\x017\n!\x08\x003',
    ],
  },
  CAR.SUBARU_OUTBACK_2023: {
    (Ecu.abs, 0x7b0, None): [
      b'\xa1 #\x14\x00',
      b'\xa1 #\x17\x00',
    ],
    (Ecu.eps, 0x746, None): [
      b'+\xc0\x10\x11\x00',
      b'+\xc0\x12\x11\x00',
    ],
    (Ecu.fwdCamera, 0x787, None): [
      b'\t!\x08\x046\x05!\x08\x01/',
    ],
  },
}
