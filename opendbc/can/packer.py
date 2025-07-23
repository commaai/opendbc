import re
import os
import math
from dataclasses import dataclass
from collections.abc import Callable

DBC_CACHE: dict[str, "DBC"] = {}


class SignalType:
  DEFAULT = 0
  COUNTER = 1
  HONDA_CHECKSUM = 2
  TOYOTA_CHECKSUM = 3
  BODY_CHECKSUM = 4
  VOLKSWAGEN_MQB_MEB_CHECKSUM = 5
  XOR_CHECKSUM = 6
  SUBARU_CHECKSUM = 7
  CHRYSLER_CHECKSUM = 8
  HKG_CAN_FD_CHECKSUM = 9
  FCA_GIORGIO_CHECKSUM = 10
  TESLA_CHECKSUM = 11


@dataclass
class Signal:
  name: str
  start_bit: int
  msb: int
  lsb: int
  size: int
  is_signed: bool
  factor: float
  offset: float
  is_little_endian: bool
  type: int = SignalType.DEFAULT
  calc_checksum: 'Callable[[int, Signal, bytearray], int] | None' = None

@dataclass
class Msg:
  name: str
  address: int
  size: int
  sigs: dict[str, Signal]


@dataclass
class DBC:
  name: str
  msgs: dict[int, Msg]
  addr_to_msg: dict[int, Msg]
  name_to_msg: dict[str, Msg]


# ***** checksum functions *****


def honda_checksum(address: int, sig: Signal, d: bytearray) -> int:
  s = 0
  extended = address > 0x7FF
  addr = address
  while addr:
    s += addr & 0xF
    addr >>= 4
  for i in range(len(d)):
    x = d[i]
    if i == len(d) - 1:
      x >>= 4
    s += (x & 0xF) + (x >> 4)
  s = 8 - s
  if extended:
    s += 3
  return s & 0xF


def toyota_checksum(address: int, sig: Signal, d: bytearray) -> int:
  s = len(d)
  addr = address
  while addr:
    s += addr & 0xFF
    addr >>= 8
  for i in range(len(d) - 1):
    s += d[i]
  return s & 0xFF


def subaru_checksum(address: int, sig: Signal, d: bytearray) -> int:
  s = 0
  addr = address
  while addr:
    s += addr & 0xFF
    addr >>= 8
  for i in range(1, len(d)):
    s += d[i]
  return s & 0xFF


def chrysler_checksum(address: int, sig: Signal, d: bytearray) -> int:
  checksum = 0xFF
  for j in range(len(d) - 1):
    curr = d[j]
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


def xor_checksum(address: int, sig: Signal, d: bytearray) -> int:
  checksum = 0
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum ^= d[i]
  return checksum


def body_checksum(address: int, sig: Signal, d: bytearray) -> int:
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


def volkswagen_mqb_meb_checksum(address: int, sig: Signal, d: bytearray) -> int:
  crc = 0xFF
  for i in range(1, len(d)):
    crc ^= d[i]
    crc = CRC8H2F[crc]
  counter = d[1] & 0x0F
  const = VOLKSWAGEN_MQB_MEB_CONSTANTS.get(address)
  if const:
    crc ^= const[counter]
    crc = CRC8H2F[crc]
  else:
    pass
  return crc ^ 0xFF


# crc lookup tables
CRC8H2F = []
CRC8J1850 = []
CRC16_XMODEM = []


def _gen_crc8_table(poly: int) -> list[int]:
  table = []
  for i in range(256):
    crc = i
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
    table.append(crc)
  return table


def _gen_crc16_table(poly: int) -> list[int]:
  table = []
  for i in range(256):
    crc = i << 8
    for _ in range(8):
      if crc & 0x8000:
        crc = ((crc << 1) ^ poly) & 0xFFFF
      else:
        crc = (crc << 1) & 0xFFFF
    table.append(crc)
  return table


CRC8H2F = _gen_crc8_table(0x2F)
CRC8J1850 = _gen_crc8_table(0x1D)
CRC16_XMODEM = _gen_crc16_table(0x1021)

VOLKSWAGEN_MQB_MEB_CONSTANTS: dict[int, list[int]] = {
  0x40: [0x40] * 16,
  0x86: [0x86] * 16,
  0x9F: [0xF5] * 16,
  0xAD: [0x3F, 0x69, 0x39, 0xDC, 0x94, 0xF9, 0x14, 0x64, 0xD8, 0x6A, 0x34, 0xCE, 0xA2, 0x55, 0xB5, 0x2C],
  0x0DB: [0x09, 0xFA, 0xCA, 0x8E, 0x62, 0xD5, 0xD1, 0xF0, 0x31, 0xA0, 0xAF, 0xDA, 0x4D, 0x1A, 0x0A, 0x97],
  0xFC: [0x77, 0x5C, 0xA0, 0x89, 0x4B, 0x7C, 0xBB, 0xD6, 0x1F, 0x6C, 0x4F, 0xF6, 0x20, 0x2B, 0x43, 0xDD],
  0xFD: [0xB4, 0xEF, 0xF8, 0x49, 0x1E, 0xE5, 0xC2, 0xC0, 0x97, 0x19, 0x3C, 0xC9, 0xF1, 0x98, 0xD6, 0x61],
  0x101: [0xAA] * 16,
  0x102: [0xD7, 0x12, 0x85, 0x7E, 0x0B, 0x34, 0xFA, 0x16, 0x7A, 0x25, 0x2D, 0x8F, 0x04, 0x8E, 0x5D, 0x35],
  0x106: [0x07] * 16,
  0x10B: [0x77, 0x5C, 0xA0, 0x89, 0x4B, 0x7C, 0xBB, 0xD6, 0x1F, 0x6C, 0x4F, 0xF6, 0x20, 0x2B, 0x43, 0xDD],
}


def hkg_can_fd_checksum(address: int, sig: Signal, d: bytearray) -> int:
  crc = 0
  for i in range(2, len(d)):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ d[i]]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 0) & 0xFF)]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 8) & 0xFF)]) & 0xFFFF
  if len(d) == 8:
    crc ^= 0x5F29
  elif len(d) == 16:
    crc ^= 0x041D
  elif len(d) == 24:
    crc ^= 0x819D
  elif len(d) == 32:
    crc ^= 0x9F5B
  return crc


def fca_giorgio_checksum(address: int, sig: Signal, d: bytearray) -> int:
  crc = 0
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8J1850[crc]
  if address == 0xDE:
    return crc ^ 0x10
  elif address == 0x106:
    return crc ^ 0xF6
  elif address == 0x122:
    return crc ^ 0xF1
  else:
    return crc ^ 0x0A


def tesla_checksum(address: int, sig: Signal, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF


def tesla_setup_signal(sig: Signal, dbc_name: str, line_num: int) -> None:
  if sig.name.endswith("Counter"):
    sig.type = SignalType.COUNTER
  elif sig.name.endswith("Checksum"):
    sig.type = SignalType.TESLA_CHECKSUM
    sig.calc_checksum = tesla_checksum


# ***** DBC parser *****
BO_RE = re.compile(r"^BO_ (\w+) (\w+) *: (\w+) (\w+)")
SG_RE = re.compile(r"^SG_ (\w+) : (\d+)\|(\d+)@(\d)([+-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[[0-9.+\-eE]+\|[0-9.+\-eE]+\] \".*\" .*")
SGM_RE = re.compile(r"^SG_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d)([+-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[[0-9.+\-eE]+\|[0-9.+\-eE]+\] \".*\" .*")


def parse_dbc(path: str) -> DBC:
  name = os.path.basename(path).replace(".dbc", "")
  with open(path) as f:
    lines = f.readlines()

  checksum_state = get_checksum_state(name)
  be_bits = [j + i * 8 for i in range(64) for j in range(7, -1, -1)]
  msgs: dict[int, Msg] = {}
  addr_to_msg: dict[int, Msg] = {}
  name_to_msg: dict[str, Msg] = {}
  address = 0
  signals_temp: dict[int, dict[str, Signal]] = {}
  for line_num, line in enumerate(lines, 1):
    line = line.strip()
    if line.startswith("BO_ "):
      m = BO_RE.match(line)
      if not m:
        continue
      address = int(m.group(1), 0)
      msg_name = m.group(2)
      size = int(m.group(3), 0)
      sigs = {}
      msgs[address] = Msg(msg_name, address, size, sigs)
      addr_to_msg[address] = msgs[address]
      name_to_msg[msg_name] = msgs[address]
      signals_temp[address] = sigs
    elif line.startswith("SG_ "):
      m = SG_RE.search(line)
      offset = 0
      if not m:
        m = SGM_RE.search(line)
        if not m:
          continue
        offset = 1
      sig_name = m.group(1)
      start_bit = int(m.group(2 + offset))
      size = int(m.group(3 + offset))
      is_little_endian = m.group(4 + offset) == "1"
      is_signed = m.group(5 + offset) == "-"
      factor = float(m.group(6 + offset))
      offset_val = float(m.group(7 + offset))

      if is_little_endian:
        lsb = start_bit
        msb = start_bit + size - 1
      else:
        idx = be_bits.index(start_bit)
        lsb = be_bits[idx + size - 1]
        msb = start_bit

      sig = Signal(sig_name, start_bit, msb, lsb, size, is_signed, factor, offset_val, is_little_endian)
      set_signal_type(sig, checksum_state, name, line_num)
      signals_temp[address][sig_name] = sig
  for addr, sigs in signals_temp.items():
    msgs[addr].sigs = sigs
  dbc = DBC(name, msgs, addr_to_msg, name_to_msg)
  return dbc


@dataclass
class ChecksumState:
  checksum_size: int
  counter_size: int
  checksum_start_bit: int
  counter_start_bit: int
  little_endian: bool
  checksum_type: int
  calc_checksum: Callable[[int, Signal, bytearray], int] | None
  setup_signal: Callable[[Signal, str, int], None] | None = None


def get_checksum_state(dbc_name: str) -> ChecksumState | None:
  if dbc_name.startswith(("honda_", "acura_")):
    return ChecksumState(4, 2, 3, 5, False, SignalType.HONDA_CHECKSUM, honda_checksum)
  elif dbc_name.startswith(("toyota_", "lexus_")):
    return ChecksumState(8, -1, 7, -1, False, SignalType.TOYOTA_CHECKSUM, toyota_checksum)
  elif dbc_name.startswith("hyundai_canfd_generated"):
    return ChecksumState(16, -1, 0, -1, True, SignalType.HKG_CAN_FD_CHECKSUM, hkg_can_fd_checksum)
  elif dbc_name.startswith(("vw_mqb", "vw_mqbevo", "vw_meb")):
    return ChecksumState(8, 4, 0, 0, True, SignalType.VOLKSWAGEN_MQB_MEB_CHECKSUM, volkswagen_mqb_meb_checksum)
  elif dbc_name.startswith("vw_pq"):
    return ChecksumState(8, 4, 0, -1, True, SignalType.XOR_CHECKSUM, xor_checksum)
  elif dbc_name.startswith("subaru_global_"):
    return ChecksumState(8, -1, 0, -1, True, SignalType.SUBARU_CHECKSUM, subaru_checksum)
  elif dbc_name.startswith("chrysler_"):
    return ChecksumState(8, -1, 7, -1, False, SignalType.CHRYSLER_CHECKSUM, chrysler_checksum)
  elif dbc_name.startswith("fca_giorgio"):
    return ChecksumState(8, -1, 7, -1, False, SignalType.FCA_GIORGIO_CHECKSUM, fca_giorgio_checksum)
  elif dbc_name.startswith("comma_body"):
    return ChecksumState(8, 4, 7, 3, False, SignalType.BODY_CHECKSUM, body_checksum)
  elif dbc_name.startswith("tesla_model3_party"):
    return ChecksumState(8, -1, 0, -1, True, SignalType.TESLA_CHECKSUM, tesla_checksum, tesla_setup_signal)
  return None


def set_signal_type(sig: Signal, chk: ChecksumState | None, dbc_name: str, line_num: int) -> None:
  sig.calc_checksum = None
  if chk:
    if chk.setup_signal:
      chk.setup_signal(sig, dbc_name, line_num)
    if sig.name == "CHECKSUM":
      sig.type = chk.checksum_type
      sig.calc_checksum = chk.calc_checksum
    elif sig.name == "COUNTER":
      sig.type = SignalType.COUNTER


# ***** packer *****
class CANPacker:
  def __init__(self, dbc_name: str):
    # Handle both string and bytes input (for compatibility with CANParser.dbc_name)
    if isinstance(dbc_name, bytes):
      dbc_name = dbc_name.decode("utf-8")
    dbc_path = dbc_name
    if not os.path.exists(dbc_path):
      dbc_path = os.path.join(os.path.dirname(__file__), "..", "dbc", dbc_name + ".dbc")
    if dbc_name in DBC_CACHE:
      self.dbc = DBC_CACHE[dbc_name]
    else:
      try:
        self.dbc = parse_dbc(dbc_path)
        DBC_CACHE[dbc_name] = self.dbc
      except FileNotFoundError as e:
        raise RuntimeError(f"DBC file not found: {dbc_path}") from e
    self.counters: dict[int, int] = {}

  def pack(self, address: int, values: dict[str, float]) -> bytearray:
    msg = self.dbc.addr_to_msg.get(address)
    if msg is None:
      return bytearray()
    dat = bytearray(msg.size)
    counter_set = False
    for name, value in values.items():
      sig = msg.sigs.get(name)
      if sig is None:
        continue
      ival = int(math.floor((value - sig.offset) / sig.factor + 0.5))
      if ival < 0:
        ival = (1 << sig.size) + ival
      set_value(dat, sig, ival)
      if sig.type == SignalType.COUNTER or sig.name == "COUNTER":
        self.counters[address] = int(value)
        counter_set = True
    sig_counter = next((s for s in msg.sigs.values() if s.type == SignalType.COUNTER or s.name == "COUNTER"), None)
    if sig_counter and not counter_set:
      if address not in self.counters:
        self.counters[address] = 0
      set_value(dat, sig_counter, self.counters[address])
      self.counters[address] = (self.counters[address] + 1) % (1 << sig_counter.size)
    sig_checksum = next((s for s in msg.sigs.values() if s.type > SignalType.COUNTER), None)
    if sig_checksum and sig_checksum.calc_checksum:
      checksum = sig_checksum.calc_checksum(address, sig_checksum, dat)
      set_value(dat, sig_checksum, checksum)
    return dat

  def make_can_msg(self, name_or_addr, bus: int, values: dict[str, float]):
    if isinstance(name_or_addr, int):
      addr = name_or_addr
    else:
      msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        return 0, b'', bus
      addr = msg.address
    dat = self.pack(addr, values)
    if len(dat) == 0:
      return 0, b'', bus
    return addr, bytes(dat), bus


def set_value(msg: bytearray, sig: Signal, ival: int) -> None:
  i = sig.lsb // 8
  bits = sig.size
  if sig.size < 64:
    ival &= (1 << sig.size) - 1
  while 0 <= i < len(msg) and bits > 0:
    shift = sig.lsb % 8 if (sig.lsb // 8) == i else 0
    size = min(bits, 8 - shift)
    mask = ((1 << size) - 1) << shift
    msg[i] &= ~mask
    msg[i] |= (ival & ((1 << size) - 1)) << shift
    bits -= size
    ival >>= size
    i = i + 1 if sig.is_little_endian else i - 1
