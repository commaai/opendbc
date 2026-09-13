import re
import os
from collections.abc import Callable
from dataclasses import dataclass, replace
from functools import cache

from opendbc import DBC_PATH, get_generated_dbcs

# TODO: these should just be passed in along with the DBC file
from opendbc.car.ford.fordcan import ford_checksum
from opendbc.car.hyundai.hyundaican import hyundai_rx_checksum
from opendbc.car.rivian.riviancan import rivian_checksum
from opendbc.car.honda.hondacan import honda_checksum
from opendbc.car.toyota.toyotacan import toyota_checksum
from opendbc.car.subaru.subarucan import subaru_checksum
from opendbc.car.chrysler.chryslercan import chrysler_checksum, fca_giorgio_checksum
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.volkswagen.mlbcan import volkswagen_mlb_checksum
from opendbc.car.volkswagen.mqbcan import volkswagen_meb_alt_crc_checksum, volkswagen_mqb_meb_checksum, xor_checksum
from opendbc.car.tesla.teslacan import tesla_checksum
from opendbc.car.body.bodycan import body_checksum
from opendbc.car.psa.psacan import psa_checksum


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
  PSA_CHECKSUM = 12
  VOLKSWAGEN_MLB_CHECKSUM = 13
  FORD_CHECKSUM = 14
  HYUNDAI_CHECKSUM = 15
  RIVIAN_CHECKSUM = 16


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
  counter_max: int | None = None
  bit_positions: tuple[int, ...] | None = None

  @property
  def counter_modulus(self) -> int:
    return self.counter_max + 1 if self.counter_max is not None else 1 << self.size


@dataclass
class Msg:
  name: str
  address: int
  size: int
  sigs: dict[str, Signal]


@dataclass
class Val:
  name: str
  address: int
  def_val: str


BO_RE = re.compile(r"^BO_ (\w+) (\w+) *: (\w+) (\w+)")
SG_RE = re.compile(r"^SG_ (\w+) : (\d+)\|(\d+)@(\d)([+-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[[0-9.+\-eE]+\|[0-9.+\-eE]+\] \".*\" .*")
SGM_RE = re.compile(r"^SG_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d)([+-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[[0-9.+\-eE]+\|[0-9.+\-eE]+\] \".*\" .*")
VAL_RE = re.compile(r"^VAL_ (\w+) (\w+) (.*);")
VAL_SPLIT_RE = re.compile(r'["]+')


@cache
class DBC:
  def __init__(self, name: str):
    if os.path.exists(name):
      self._parse_file(name)
    else:
      dbc_path = os.path.join(DBC_PATH, name + ".dbc")
      if content := get_generated_dbcs().get(name):
        self._parse_content(name, content)
      elif os.path.exists(dbc_path):
        self._parse_file(dbc_path)
      else:
        raise FileNotFoundError(f"DBC not found: {name}")

  def _parse_file(self, path: str):
    self.name = os.path.basename(path).replace(".dbc", "")
    with open(path) as f:
      lines = f.readlines()
    self._parse_lines(lines)

  def _parse_content(self, name: str, content: str):
    self.name = name
    lines = content.splitlines(keepends=True)
    self._parse_lines(lines)

  def _parse_lines(self, lines: list[str]):

    checksum_state = get_checksum_state(self.name)
    be_bits = [j + i * 8 for i in range(64) for j in range(7, -1, -1)]
    self.msgs: dict[int, Msg] = {}
    self.addr_to_msg: dict[int, Msg] = {}
    self.name_to_msg: dict[str, Msg] = {}
    self.vals: list[Val] = []
    address = 0
    signals_temp: dict[int, dict[str, Signal]] = {}
    for line in lines:
      line = line.strip()
      if line.startswith("BO_ "):
        m = BO_RE.match(line)
        if not m:
          continue
        address = int(m.group(1), 0)
        msg_name = m.group(2)
        size = int(m.group(3), 0)
        sigs = {}
        self.msgs[address] = Msg(msg_name, address, size, sigs)
        self.addr_to_msg[address] = self.msgs[address]
        self.name_to_msg[msg_name] = self.msgs[address]
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
        set_signal_type(sig, checksum_state, address)
        signals_temp[address][sig_name] = sig
      elif line.startswith("VAL_ "):
        m = VAL_RE.search(line)
        if not m:
          continue
        val_addr = int(m.group(1), 0)
        sgname = m.group(2)
        defs = m.group(3)
        words = [w.strip() for w in VAL_SPLIT_RE.split(defs) if w.strip()]
        words = [w.upper().replace(" ", "_") for w in words]
        val_def = " ".join(words).strip()
        self.vals.append(Val(sgname, val_addr, val_def))
    for addr, sigs in signals_temp.items():
      if self.name == "hyundai_can_generated" and addr == 0x386:
        # WHL_SPD11 splits each four-bit validation field across two bytes.
        for name, source, positions in (
          ("COUNTER", "WHL_SPD_AliveCounter_LSB", (14, 15, 30, 31)),
          ("CHECKSUM", "WHL_SPD_Checksum_LSB", (46, 47, 62, 63)),
        ):
          sig = replace(sigs[source], name=name, size=4, bit_positions=positions)
          set_signal_type(sig, checksum_state, addr)
          sigs[name] = sig
      self.msgs[addr].sigs = sigs


# ***** checksum functions *****

def tesla_setup_signal(sig: Signal, address: int) -> None:
  if sig.name.endswith("Counter"):
    sig.type = SignalType.COUNTER
  elif sig.name.endswith("Checksum"):
    sig.type = SignalType.TESLA_CHECKSUM
    sig.calc_checksum = tesla_checksum


def ford_setup_signal(sig: Signal, address: int) -> None:
  if address in (0x415, 0x91):
    if sig.name.endswith("_No_Cnt"):
      sig.type = SignalType.COUNTER
    elif sig.name.endswith("_No_Cs"):
      sig.type = SignalType.FORD_CHECKSUM
      sig.calc_checksum = ford_checksum


def hyundai_setup_signal(sig: Signal, address: int) -> None:
  signals = {
    0x260: ("AliveCounter", "Checksum"),
    0x4F1: ("CF_Clu_AliveCnt1", None),
    0x421: ("CR_VSM_Alive", "CR_VSM_ChkSum"),
    0x394: ("AliveCounterTCS", "CheckSum_TCS3"),
  }
  counter, checksum = signals.get(address, (None, None))
  if sig.name == counter:
    sig.type = SignalType.COUNTER
  elif sig.name == checksum:
    sig.type = SignalType.HYUNDAI_CHECKSUM
    sig.calc_checksum = hyundai_rx_checksum


def rivian_setup_signal(sig: Signal, address: int) -> None:
  if address in (0x208, 0x150, 0x38F, 0x380, 0x100):
    if sig.name.endswith(("_Counter", "_AliveCounter")):
      sig.type = SignalType.COUNTER
      sig.counter_max = 14
    elif sig.name.endswith("_Checksum"):
      sig.type = SignalType.RIVIAN_CHECKSUM
      sig.calc_checksum = rivian_checksum


@dataclass
class ChecksumState:
  checksum_type: int
  calc_checksum: Callable[[int, Signal, bytearray], int] | None
  setup_signal: Callable[[Signal, int], None] | None = None


def get_checksum_state(dbc_name: str) -> ChecksumState | None:
  if dbc_name.startswith("ford_"):
    return ChecksumState(SignalType.FORD_CHECKSUM, ford_checksum, ford_setup_signal)
  elif dbc_name == "hyundai_can_generated":
    return ChecksumState(SignalType.HYUNDAI_CHECKSUM, hyundai_rx_checksum, hyundai_setup_signal)
  elif dbc_name == "rivian_primary_actuator":
    return ChecksumState(SignalType.RIVIAN_CHECKSUM, rivian_checksum, rivian_setup_signal)
  elif dbc_name.startswith(("honda_", "acura_")):
    return ChecksumState(SignalType.HONDA_CHECKSUM, honda_checksum)
  elif dbc_name.startswith(("toyota_", "lexus_")):
    return ChecksumState(SignalType.TOYOTA_CHECKSUM, toyota_checksum)
  elif dbc_name.startswith("hyundai_canfd_generated"):
    return ChecksumState(SignalType.HKG_CAN_FD_CHECKSUM, hkg_can_fd_checksum)
  elif dbc_name.startswith("vw_meb_2024"):
    return ChecksumState(SignalType.VOLKSWAGEN_MQB_MEB_CHECKSUM, volkswagen_meb_alt_crc_checksum)
  elif dbc_name.startswith(("vw_mqb", "vw_mqbevo", "vw_meb")):
    return ChecksumState(SignalType.VOLKSWAGEN_MQB_MEB_CHECKSUM, volkswagen_mqb_meb_checksum)
  elif dbc_name.startswith("vw_mlb"):
    return ChecksumState(SignalType.VOLKSWAGEN_MLB_CHECKSUM, volkswagen_mlb_checksum)
  elif dbc_name.startswith("vw_pq"):
    return ChecksumState(SignalType.XOR_CHECKSUM, xor_checksum)
  elif dbc_name.startswith("subaru_global_"):
    return ChecksumState(SignalType.SUBARU_CHECKSUM, subaru_checksum)
  elif dbc_name.startswith("chrysler_"):
    return ChecksumState(SignalType.CHRYSLER_CHECKSUM, chrysler_checksum)
  elif dbc_name.startswith("fca_giorgio"):
    return ChecksumState(SignalType.FCA_GIORGIO_CHECKSUM, fca_giorgio_checksum)
  elif dbc_name.startswith("comma_body"):
    return ChecksumState(SignalType.BODY_CHECKSUM, body_checksum)
  elif dbc_name.startswith("tesla_model3_party"):
    return ChecksumState(SignalType.TESLA_CHECKSUM, tesla_checksum, tesla_setup_signal)
  elif dbc_name.startswith("psa_"):
    return ChecksumState(SignalType.PSA_CHECKSUM, psa_checksum)
  return None


def set_signal_type(sig: Signal, chk: ChecksumState | None, address: int) -> None:
  sig.calc_checksum = None
  if chk:
    if chk.setup_signal:
      chk.setup_signal(sig, address)
    if sig.name == "CHECKSUM":
      sig.type = chk.checksum_type
      sig.calc_checksum = chk.calc_checksum
    elif sig.name == "COUNTER":
      sig.type = SignalType.COUNTER
