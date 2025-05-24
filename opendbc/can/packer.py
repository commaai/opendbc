from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional, Tuple

from opendbc import DBC_PATH


class SignalType(IntEnum):
    DEFAULT = 0
    COUNTER = 1
    HONDA_CHECKSUM = 2
    TOYOTA_CHECKSUM = 3
    PEDAL_CHECKSUM = 4
    VOLKSWAGEN_MQB_MEB_CHECKSUM = 5
    XOR_CHECKSUM = 6
    SUBARU_CHECKSUM = 7
    CHRYSLER_CHECKSUM = 8
    HKG_CAN_FD_CHECKSUM = 9
    FCA_GIORGIO_CHECKSUM = 10
    TESLA_CHECKSUM = 11


ChecksumFn = Callable[[int, "Signal", bytearray], int]


@dataclass
class Signal:
    name: str
    start_bit: int
    size: int
    is_little_endian: bool
    is_signed: bool
    factor: float
    offset: float
    lsb: int = 0
    msb: int = 0
    type: SignalType = SignalType.DEFAULT
    calc_checksum: Optional[ChecksumFn] = None


@dataclass
class Msg:
    name: str
    address: int
    size: int
    sigs: List[Signal] = field(default_factory=list)


@dataclass
class ChecksumState:
    checksum_size: int
    counter_size: int
    checksum_start_bit: int
    counter_start_bit: int
    little_endian: bool
    checksum_type: SignalType
    calc_checksum: Optional[ChecksumFn] = None
    setup_signal: Optional[Callable[[Signal], None]] = None


def pedal_setup_signal(sig: Signal) -> None:
    if sig.name == "CHECKSUM_PEDAL":
        sig.type = SignalType.PEDAL_CHECKSUM
    elif sig.name == "COUNTER_PEDAL":
        sig.type = SignalType.COUNTER


def tesla_setup_signal(sig: Signal) -> None:
    if sig.name.endswith("Counter"):
        sig.type = SignalType.COUNTER
    elif sig.name.endswith("Checksum"):
        sig.type = SignalType.TESLA_CHECKSUM
        sig.calc_checksum = tesla_checksum


def honda_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    s = 0
    extended = addr > 0x7FF
    a = addr
    while a:
        s += a & 0xF
        a >>= 4
    for i, x in enumerate(d):
        if i == len(d) - 1:
            x >>= 4
        s += (x & 0xF) + (x >> 4)
    s = 8 - s
    if extended:
        s += 3
    return s & 0xF


def toyota_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    s = len(d)
    a = addr
    while a:
        s += a & 0xFF
        a >>= 8
    for b in d[:-1]:
        s += b
    return s & 0xFF


def subaru_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    s = 0
    a = addr
    while a:
        s += a & 0xFF
        a >>= 8
    for b in d[1:]:
        s += b
    return s & 0xFF


def chrysler_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    checksum = 0xFF
    for curr in d[:-1]:
        shift = 0x80
        for _ in range(8):
            bit_sum = curr & shift
            temp_chk = checksum & 0x80
            if bit_sum:
                bit_sum = 0x1C
                if temp_chk:
                    bit_sum = 1
                checksum = (checksum << 1) & 0xFF
                temp_chk = checksum | 1
                bit_sum ^= temp_chk
            else:
                if temp_chk:
                    bit_sum = 0x1D
                checksum = (checksum << 1) & 0xFF
                bit_sum ^= checksum
            checksum = bit_sum & 0xFF
            shift >>= 1
    return (~checksum) & 0xFF


crc8_lut_8h2f: List[int] = [0] * 256
crc8_lut_j1850: List[int] = [0] * 256
crc16_lut_xmodem: List[int] = [0] * 256


def _gen_crc_lookup_table_8(poly: int, lut: List[int]) -> None:
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        lut[i] = crc


def _gen_crc_lookup_table_16(poly: int, lut: List[int]) -> None:
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        lut[i] = crc


_gen_crc_lookup_table_8(0x2F, crc8_lut_8h2f)
_gen_crc_lookup_table_8(0x1D, crc8_lut_j1850)
_gen_crc_lookup_table_16(0x1021, crc16_lut_xmodem)


VOLKSWAGEN_MQB_MEB_CRC_CONSTANTS: Dict[int, Tuple[int, ...]] = {
    0x40:  (0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40),
    0x86:  (0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86),
    0x9F:  (0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5),
    0xAD:  (0x3F, 0x69, 0x39, 0xDC, 0x94, 0xF9, 0x14, 0x64, 0xD8, 0x6A, 0x34, 0xCE, 0xA2, 0x55, 0xB5, 0x2C),
    0x0DB: (0x09, 0xFA, 0xCA, 0x8E, 0x62, 0xD5, 0xD1, 0xF0, 0x31, 0xA0, 0xAF, 0xDA, 0x4D, 0x1A, 0x0A, 0x97),
    0xFC:  (0x77, 0x5C, 0xA0, 0x89, 0x4B, 0x7C, 0xBB, 0xD6, 0x1F, 0x6C, 0x4F, 0xF6, 0x20, 0x2B, 0x43, 0xDD),
    0xFD:  (0xB4, 0xEF, 0xF8, 0x49, 0x1E, 0xE5, 0xC2, 0xC0, 0x97, 0x19, 0x3C, 0xC9, 0xF1, 0x98, 0xD6, 0x61),
    0x101: (0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA),
    0x102: (0xD7, 0x12, 0x85, 0x7E, 0x0B, 0x34, 0xFA, 0x16, 0x7A, 0x25, 0x2D, 0x8F, 0x04, 0x8E, 0x5D, 0x35),
    0x106: (0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07),
    0x10B: (0x77, 0x5C, 0xA0, 0x89, 0x4B, 0x7C, 0xBB, 0xD6, 0x1F, 0x6C, 0x4F, 0xF6, 0x20, 0x2B, 0x43, 0xDD),
    0x116: (0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC, 0xAC),
    0x117: (0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16),
    0x120: (0xC4, 0xE2, 0x4F, 0xE4, 0xF8, 0x2F, 0x56, 0x81, 0x9F, 0xE5, 0x83, 0x44, 0x05, 0x3F, 0x97, 0xDF),
    0x121: (0xE9, 0x65, 0xAE, 0x6B, 0x7B, 0x35, 0xE5, 0x5F, 0x4E, 0xC7, 0x86, 0xA2, 0xBB, 0xDD, 0xEB, 0xB4),
    0x122: (0x37, 0x7D, 0xF3, 0xA9, 0x18, 0x46, 0x6D, 0x4D, 0x3D, 0x71, 0x92, 0x9C, 0xE5, 0x32, 0x10, 0xB9),
    0x126: (0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA, 0xDA),
    0x12B: (0x6A, 0x38, 0xB4, 0x27, 0x22, 0xEF, 0xE1, 0xBB, 0xF8, 0x80, 0x84, 0x49, 0xC7, 0x9E, 0x1E, 0x2B),
    0x12E: (0xF8, 0xE5, 0x97, 0xC9, 0xD6, 0x07, 0x47, 0x21, 0x66, 0xDD, 0xCF, 0x6F, 0xA1, 0x94, 0x74, 0x63),
    0x139: (0xED, 0x03, 0x1C, 0x13, 0xC6, 0x23, 0x78, 0x7A, 0x8B, 0x40, 0x14, 0x51, 0xBF, 0x68, 0x32, 0xBA),
    0x13D: (0x20, 0xCA, 0x68, 0xD5, 0x1B, 0x31, 0xE2, 0xDA, 0x08, 0x0A, 0xD4, 0xDE, 0x9C, 0xE4, 0x35, 0x5B),
    0x14C: (0x16, 0x35, 0x59, 0x15, 0x9A, 0x2A, 0x97, 0xB8, 0x0E, 0x4E, 0x30, 0xCC, 0xB3, 0x07, 0x01, 0xAD),
    0x14D: (0x1A, 0x65, 0x81, 0x96, 0xC0, 0xDF, 0x11, 0x92, 0xD3, 0x61, 0xC6, 0x95, 0x8C, 0x29, 0x21, 0xB5),
    0x187: (0x7F, 0xED, 0x17, 0xC2, 0x7C, 0xEB, 0x44, 0x21, 0x01, 0xFA, 0xDB, 0x15, 0x4A, 0x6B, 0x23, 0x05),
    0x1A4: (0x69, 0xBB, 0x54, 0xE6, 0x4E, 0x46, 0x8D, 0x7B, 0xEA, 0x87, 0xE9, 0xB3, 0x63, 0xCE, 0xF8, 0xBF),
    0x1AB: (0x13, 0x21, 0x9B, 0x6A, 0x9A, 0x62, 0xD4, 0x65, 0x18, 0xF1, 0xAB, 0x16, 0x32, 0x89, 0xE7, 0x26),
    0x1F0: (0x2F, 0x3C, 0x22, 0x60, 0x18, 0xEB, 0x63, 0x76, 0xC5, 0x91, 0x0F, 0x27, 0x34, 0x04, 0x7F, 0x02),
    0x20A: (0x9D, 0xE8, 0x36, 0xA1, 0xCA, 0x3B, 0x1D, 0x33, 0xE0, 0xD5, 0xBB, 0x5F, 0xAE, 0x3C, 0x31, 0x9F),
    0x26B: (0xCE, 0xCC, 0xBD, 0x69, 0xA1, 0x3C, 0x18, 0x76, 0x0F, 0x04, 0xF2, 0x3A, 0x93, 0x24, 0x19, 0x51),
    0x30C: (0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F),
    0x30F: (0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C),
    0x324: (0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27),
    0x3BE: (0x1F, 0x28, 0xC6, 0x85, 0xE6, 0xF8, 0xB0, 0x19, 0x5B, 0x64, 0x35, 0x21, 0xE4, 0xF7, 0x9C, 0x24),
    0x3C0: (0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3),
    0x3D5: (0xC5, 0x39, 0xC7, 0xF9, 0x92, 0xD8, 0x24, 0xCE, 0xF1, 0xB5, 0x7A, 0xC4, 0xBC, 0x60, 0xE3, 0xD1),
    0x65D: (0xAC, 0xB3, 0xAB, 0xEB, 0x7A, 0xE1, 0x3B, 0xF7, 0x73, 0xBA, 0x7C, 0x9E, 0x06, 0x5F, 0x02, 0xD9),
}


def volkswagen_mqb_meb_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    crc = 0xFF
    for b in d[1:]:
        crc ^= b
        crc = crc8_lut_8h2f[crc]
    counter = d[1] & 0x0F
    const = VOLKSWAGEN_MQB_MEB_CRC_CONSTANTS.get(addr)
    if const is not None:
        crc ^= const[counter]
        crc = crc8_lut_8h2f[crc]
    return crc ^ 0xFF


def xor_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    checksum = 0
    checksum_byte = sig.start_bit // 8
    for i, b in enumerate(d):
        if i != checksum_byte:
            checksum ^= b
    return checksum


def pedal_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    crc = 0xFF
    poly = 0xD5
    for i in range(len(d) - 2, -1, -1):
        crc ^= d[i]
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def hkg_can_fd_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    crc = 0
    for b in d[2:]:
        crc = ((crc << 8) ^ crc16_lut_xmodem[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    crc = ((crc << 8) ^ crc16_lut_xmodem[((crc >> 8) ^ (addr & 0xFF)) & 0xFF]) & 0xFFFF
    crc = ((crc << 8) ^ crc16_lut_xmodem[((crc >> 8) ^ ((addr >> 8) & 0xFF)) & 0xFF]) & 0xFFFF
    if len(d) == 8:
        crc ^= 0x5F29
    elif len(d) == 16:
        crc ^= 0x041D
    elif len(d) == 24:
        crc ^= 0x819D
    elif len(d) == 32:
        crc ^= 0x9F5B
    return crc & 0xFFFF


def fca_giorgio_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    crc = 0x00
    for b in d[:-1]:
        crc ^= b
        crc = crc8_lut_j1850[crc]
    if addr == 0xDE:
        return crc ^ 0x10
    if addr == 0x106:
        return crc ^ 0xF6
    if addr == 0x122:
        return crc ^ 0xF1
    return crc ^ 0x0A


def tesla_checksum(addr: int, sig: Signal, d: bytearray) -> int:
    checksum = (addr & 0xFF) + ((addr >> 8) & 0xFF)
    checksum_byte = sig.start_bit // 8
    for i, b in enumerate(d):
        if i != checksum_byte:
            checksum += b
    return checksum & 0xFF


CHECKSUM_MAP = {
    SignalType.HONDA_CHECKSUM: honda_checksum,
    SignalType.TOYOTA_CHECKSUM: toyota_checksum,
    SignalType.PEDAL_CHECKSUM: pedal_checksum,
    SignalType.VOLKSWAGEN_MQB_MEB_CHECKSUM: volkswagen_mqb_meb_checksum,
    SignalType.XOR_CHECKSUM: xor_checksum,
    SignalType.SUBARU_CHECKSUM: subaru_checksum,
    SignalType.CHRYSLER_CHECKSUM: chrysler_checksum,
    SignalType.HKG_CAN_FD_CHECKSUM: hkg_can_fd_checksum,
    SignalType.FCA_GIORGIO_CHECKSUM: fca_giorgio_checksum,
    SignalType.TESLA_CHECKSUM: tesla_checksum,
}


BE_BITS = [j + i * 8 for i in range(64) for j in range(7, -1, -1)]


SG_RE = re.compile(r"^SG_ +(?P<name>\w+)(?: +\w+)? *: *(?P<start>\d+)\|(?P<size>\d+)@(?P<endian>\d)(?P<sign>[+-]) *\((?P<factor>[0-9.+-eE]+),(?P<offset>[0-9.+-eE]+)\)")
BO_RE = re.compile(r"^BO_ +(\w+) +(\w+) *: +(\w+)")


class DBC:
    def __init__(self, name: str) -> None:
        self.name = name
        self.msgs: List[Msg] = []
        self.addr_to_msg: Dict[int, Msg] = {}
        self.name_to_msg: Dict[str, Msg] = {}


def get_checksum_state(dbc_name: str) -> Optional[ChecksumState]:
    if dbc_name.startswith(("honda_", "acura_")):
        return ChecksumState(4, 2, 3, 5, False, SignalType.HONDA_CHECKSUM, honda_checksum)
    if dbc_name.startswith(("toyota_", "lexus_")):
        return ChecksumState(8, -1, 7, -1, False, SignalType.TOYOTA_CHECKSUM, toyota_checksum)
    if dbc_name.startswith("hyundai_canfd_generated"):
        return ChecksumState(16, -1, 0, -1, True, SignalType.HKG_CAN_FD_CHECKSUM, hkg_can_fd_checksum)
    if dbc_name.startswith(("vw_mqb", "vw_mqbevo", "vw_meb")):
        return ChecksumState(8, 4, 0, 0, True, SignalType.VOLKSWAGEN_MQB_MEB_CHECKSUM, volkswagen_mqb_meb_checksum)
    if dbc_name.startswith("vw_pq"):
        return ChecksumState(8, 4, 0, -1, True, SignalType.XOR_CHECKSUM, xor_checksum)
    if dbc_name.startswith("subaru_global_"):
        return ChecksumState(8, -1, 0, -1, True, SignalType.SUBARU_CHECKSUM, subaru_checksum)
    if dbc_name.startswith("chrysler_"):
        return ChecksumState(8, -1, 7, -1, False, SignalType.CHRYSLER_CHECKSUM, chrysler_checksum)
    if dbc_name.startswith("fca_giorgio"):
        return ChecksumState(8, -1, 7, -1, False, SignalType.FCA_GIORGIO_CHECKSUM, fca_giorgio_checksum)
    if dbc_name.startswith("comma_body"):
        return ChecksumState(8, 4, 7, 3, False, SignalType.PEDAL_CHECKSUM, pedal_checksum)
    if dbc_name.startswith("tesla_model3_party"):
        return ChecksumState(8, -1, 0, -1, True, SignalType.TESLA_CHECKSUM, tesla_checksum, tesla_setup_signal)
    return None


def set_signal_type(sig: Signal, state: Optional[ChecksumState]) -> None:
    sig.calc_checksum = None
    if state is None:
        return
    if state.setup_signal:
        state.setup_signal(sig)
    pedal_setup_signal(sig)
    if sig.name == "CHECKSUM":
        sig.type = state.checksum_type
        sig.calc_checksum = state.calc_checksum
    elif sig.name == "COUNTER":
        sig.type = SignalType.COUNTER


class DBCParser:
    def __init__(self, dbc_name: str) -> None:
        self.dbc_name = dbc_name
        self.state = get_checksum_state(dbc_name)
        path = dbc_name
        if not os.path.isfile(path):
            path = os.path.join(DBC_PATH, f"{dbc_name}.dbc")
        with open(path, "r", encoding="utf-8") as f:
            self.lines = [line.strip() for line in f]

    def parse(self) -> DBC:
        dbc = DBC(self.dbc_name)
        current_msg: Optional[Msg] = None
        for line in self.lines:
            if line.startswith("BO_"):
                m = BO_RE.search(line)
                if not m:
                    continue
                address = int(m.group(1), 0)
                name = m.group(2)
                size = int(m.group(3), 0)
                current_msg = Msg(name, address, size)
                dbc.msgs.append(current_msg)
                dbc.addr_to_msg[address] = current_msg
                dbc.name_to_msg[name] = current_msg
            elif line.startswith("SG_") and current_msg is not None:
                m = SG_RE.search(line)
                if not m:
                    continue
                name = m.group("name")
                start = int(m.group("start"))
                size = int(m.group("size"))
                is_little = m.group("endian") == "1"
                is_signed = m.group("sign") == "-"
                factor = float(m.group("factor"))
                offset = float(m.group("offset"))
                sig = Signal(name, start, size, is_little, is_signed, factor, offset)
                if is_little:
                    sig.lsb = start
                    sig.msb = start + size - 1
                else:
                    idx = BE_BITS.index(start)
                    sig.lsb = BE_BITS[idx + size - 1]
                    sig.msb = start
                set_signal_type(sig, self.state)
                current_msg.sigs.append(sig)
        return dbc


def set_value(msg: bytearray, sig: Signal, ival: int) -> None:
    i = sig.lsb // 8
    bits = sig.size
    if sig.size < 64:
        ival &= (1 << sig.size) - 1
    while 0 <= i < len(msg) and bits > 0:
        shift = sig.lsb % 8 if i == sig.lsb // 8 else 0
        size = min(bits, 8 - shift)
        msg[i] &= ~(((1 << size) - 1) << shift)
        msg[i] |= ((ival & ((1 << size) - 1)) << shift)
        bits -= size
        ival >>= size
        i = i + 1 if sig.is_little_endian else i - 1


class CANPacker:
    def __init__(self, dbc_name: str) -> None:
        parser = DBCParser(dbc_name)
        self.dbc = parser.parse()
        self.signal_lookup: Dict[int, Dict[str, Signal]] = {}
        for msg in self.dbc.msgs:
            lookup = {s.name: s for s in msg.sigs}
            self.signal_lookup[msg.address] = lookup
        self.counters: Dict[int, int] = {}

    def pack(self, address: int, values: Dict[str, float]) -> bytearray:
        msg = self.dbc.addr_to_msg.get(address)
        if msg is None:
            return bytearray()
        ret = bytearray([0] * msg.size)
        counter_set = False
        for name, value in values.items():
            sig = self.signal_lookup.get(address, {}).get(name)
            if sig is None:
                continue
            ival = int(round((value - sig.offset) / sig.factor))
            if ival < 0:
                ival = (1 << sig.size) + ival
            set_value(ret, sig, ival)
            if sig.type == SignalType.COUNTER or sig.name == "COUNTER":
                self.counters[address] = int(value)
                counter_set = True
        # set counter if defined
        counter_sig = next((s for s in msg.sigs if s.type == SignalType.COUNTER or s.name == "COUNTER"), None)
        if counter_sig and not counter_set:
            val = self.counters.get(address, 0)
            set_value(ret, counter_sig, val)
            self.counters[address] = (val + 1) % (1 << counter_sig.size)
        # checksum
        checksum_sig = next((s for s in msg.sigs if s.type.value > SignalType.COUNTER.value), None)
        if checksum_sig and checksum_sig.calc_checksum is not None:
            checksum = checksum_sig.calc_checksum(address, checksum_sig, ret)
            set_value(ret, checksum_sig, checksum)
        return ret

    def make_can_msg(self, name_or_addr: int | str, bus: int, values: Dict[str, float]) -> Tuple[int, bytes, int]:
        if isinstance(name_or_addr, int):
            address = name_or_addr
        else:
            msg = self.dbc.name_to_msg.get(name_or_addr)
            address = msg.address if msg else 0
        dat = self.pack(address, values)
        return address, bytes(dat), bus

    def lookup_message(self, address: int) -> Optional[Msg]:
        return self.dbc.addr_to_msg.get(address)
