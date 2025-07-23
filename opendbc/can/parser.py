import os
import numbers
from collections import defaultdict, deque
from dataclasses import dataclass, field

from opendbc.can.packer import DBC, DBC_CACHE, Signal, parse_dbc


MAX_BAD_COUNTER = 5
CAN_INVALID_CNT = 5


def _get_dbc(dbc_name: str) -> DBC:
  dbc_path = dbc_name
  if not os.path.exists(dbc_path):
    dbc_path = os.path.join(os.path.dirname(__file__), "..", "dbc", dbc_name + ".dbc")
  if dbc_name in DBC_CACHE:
    return DBC_CACHE[dbc_name]
  try:
    dbc = parse_dbc(dbc_path)
  except FileNotFoundError as e:
    raise RuntimeError(f"DBC file not found: {dbc_path}") from e
  DBC_CACHE[dbc_name] = dbc
  return dbc


def get_raw_value(dat: bytes | bytearray, sig: Signal) -> int:
  ret = 0
  i = sig.msb // 8
  bits = sig.size
  while 0 <= i < len(dat) and bits > 0:
    lsb = sig.lsb if (sig.lsb // 8) == i else i * 8
    msb = sig.msb if (sig.msb // 8) == i else (i + 1) * 8 - 1
    size = msb - lsb + 1
    d = (dat[i] >> (lsb - (i * 8))) & ((1 << size) - 1)
    ret |= d << (bits - size)
    bits -= size
    i = i - 1 if sig.is_little_endian else i + 1
  return ret


@dataclass
class MessageState:
  address: int
  name: str
  size: int
  signals: list[Signal]
  ignore_alive: bool = False
  ignore_checksum: bool = False
  ignore_counter: bool = False
  frequency: float = 0.0
  timeout_threshold: float = 0.0
  vals: list[float] = field(default_factory=list)
  all_vals: list[list[float]] = field(default_factory=list)
  timestamps: deque[int] = field(default_factory=deque)
  counter: int = 0
  counter_fail: int = 0
  first_seen_nanos: int = 0

  def parse(self, nanos: int, dat: bytes) -> bool:
    tmp_vals: list[float] = [0.0] * len(self.signals)
    checksum_failed = False
    counter_failed = False

    if self.first_seen_nanos == 0:
      self.first_seen_nanos = nanos

    for i, sig in enumerate(self.signals):
      tmp = get_raw_value(dat, sig)
      if sig.is_signed:
        tmp -= ((tmp >> (sig.size - 1)) & 0x1) * (1 << sig.size)

      if not self.ignore_checksum and sig.calc_checksum is not None:
        if sig.calc_checksum(self.address, sig, bytearray(dat)) != tmp:
          checksum_failed = True

      if not self.ignore_counter and sig.type == 1:  # COUNTER
        if not self.update_counter(tmp, sig.size):
          counter_failed = True

      tmp_vals[i] = tmp * sig.factor + sig.offset

    if checksum_failed or counter_failed:
      return False

    if not self.vals:
      self.vals = [0.0] * len(self.signals)
      self.all_vals = [[] for _ in self.signals]

    for i, v in enumerate(tmp_vals):
      self.vals[i] = v
      self.all_vals[i].append(v)

    self.timestamps.append(nanos)
    max_buffer = 500
    while len(self.timestamps) > max_buffer:
      self.timestamps.popleft()

    if self.frequency < 1e-5 and len(self.timestamps) >= 3:
      dt = (self.timestamps[-1] - self.timestamps[0]) * 1e-9
      if (dt > 1.0 or len(self.timestamps) >= max_buffer) and dt != 0:
        self.frequency = min(len(self.timestamps) / dt, 100.0)
        self.timeout_threshold = (1_000_000_000 / self.frequency) * 10
    return True

  def update_counter(self, cur_count: int, cnt_size: int) -> bool:
    if ((self.counter + 1) & ((1 << cnt_size) - 1)) != cur_count:
      self.counter_fail = min(self.counter_fail + 1, MAX_BAD_COUNTER)
    elif self.counter_fail > 0:
      self.counter_fail -= 1
    self.counter = cur_count
    return self.counter_fail < MAX_BAD_COUNTER

  def valid(self, current_nanos: int, bus_timeout: bool) -> bool:
    if self.ignore_alive:
      return True
    if not self.timestamps:
      return False
    if self.timeout_threshold > 0 and (current_nanos - self.timestamps[-1]) > self.timeout_threshold:
      return False
    return True


class CANParser:
  def __init__(self, dbc_name: str, messages: list[tuple[str | int, int]], bus: int = 0):
    self.dbc_name: str = dbc_name
    self.bus: int = bus
    self.dbc: DBC | None = _get_dbc(dbc_name)
    if not self.dbc:
      raise RuntimeError(f"Can't find DBC: {dbc_name}")

    self.vl: dict[int | str, dict[str, float]] = {}
    self.vl_all: dict[int | str, dict[str, list[float]]] = {}
    self.ts_nanos: dict[int | str, dict[str, int]] = {}
    self.addresses: set[int] = set()
    self.message_states: dict[int, MessageState] = {}

    for name_or_addr, freq in messages:
      if isinstance(name_or_addr, numbers.Number):
        msg = self.dbc.addr_to_msg.get(int(name_or_addr))
      else:
        msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        raise RuntimeError(f"could not find message {name_or_addr!r} in DBC {dbc_name}")
      if msg.address in self.addresses:
        raise RuntimeError("Duplicate Message Check: %d" % msg.address)

      self.addresses.add(msg.address)
      signal_names = list(msg.sigs.keys())
      self.vl[msg.address] = {s: 0.0 for s in signal_names}
      self.vl[msg.name] = self.vl[msg.address]
      self.vl_all[msg.address] = defaultdict(list)
      self.vl_all[msg.name] = self.vl_all[msg.address]
      self.ts_nanos[msg.address] = {s: 0 for s in signal_names}
      self.ts_nanos[msg.name] = self.ts_nanos[msg.address]

      state = MessageState(
        address=msg.address,
        name=msg.name,
        size=msg.size,
        signals=list(msg.sigs.values()),
        ignore_alive=freq == 0,
      )
      if 0 < freq < 10:
        state.frequency = freq
        state.timeout_threshold = (1_000_000_000 / freq) * 10

      self.message_states[msg.address] = state

    self.can_valid: bool = False
    self.bus_timeout: bool = False
    self.can_invalid_cnt: int = CAN_INVALID_CNT
    self.last_nonempty_nanos: int = 0

  def update_valid(self, nanos: int) -> None:
    valid = True
    counters_valid = True
    for state in self.message_states.values():
      if state.counter_fail >= MAX_BAD_COUNTER:
        counters_valid = False
      if not state.valid(nanos, self.bus_timeout):
        valid = False

    self.can_invalid_cnt = 0 if valid else min(self.can_invalid_cnt + 1, CAN_INVALID_CNT)
    self.can_valid = self.can_invalid_cnt < CAN_INVALID_CNT and counters_valid

  def update_strings(self, strings, sendcan: bool = False):
    if strings and not isinstance(strings[0], list | tuple):
      strings = [strings]

    for addr in self.addresses:
      for k in self.vl_all[addr]:
        self.vl_all[addr][k].clear()

    updated_addrs: set[int] = set()
    for entry in strings:
      t = entry[0]
      frames = entry[1]
      bus_empty = True
      for address, dat, src in frames:
        if src != self.bus:
          continue
        bus_empty = False
        state = self.message_states.get(address)
        if state is None or len(dat) > 64:
          continue
        if state.parse(t, dat):
          updated_addrs.add(address)
          msgname = state.name
          for i, sig in enumerate(state.signals):
            val = state.vals[i]
            self.vl[address][sig.name] = val
            self.vl[msgname][sig.name] = val
            self.vl_all[address][sig.name] = state.all_vals[i]
            self.vl_all[msgname][sig.name] = state.all_vals[i]
            self.ts_nanos[address][sig.name] = state.timestamps[-1]
            self.ts_nanos[msgname][sig.name] = state.timestamps[-1]

      if not bus_empty:
        self.last_nonempty_nanos = t
      bus_timeout_threshold = 500 * 1_000_000
      for st in self.message_states.values():
        if st.timeout_threshold > 0:
          bus_timeout_threshold = min(bus_timeout_threshold, st.timeout_threshold)
      self.bus_timeout = (t - self.last_nonempty_nanos) > bus_timeout_threshold
      self.update_valid(t)

    return updated_addrs


class CANDefine:
  def __init__(self, dbc_name: str):
    self.dbc_name = dbc_name
    self.dbc = _get_dbc(dbc_name)
    if not self.dbc:
      raise RuntimeError(f"Can't find DBC: '{dbc_name}'")

    dv = defaultdict(dict)
    for val in self.dbc.vals:
      sgname = val.name
      address = val.address
      msg = self.dbc.addr_to_msg.get(address)
      if msg is None:
        raise KeyError(address)
      msgname = msg.name
      parts = val.def_val.split()
      values = [int(v) for v in parts[::2]]
      defs = parts[1::2]
      dv[address][sgname] = dict(zip(values, defs, strict=True))
      dv[msgname][sgname] = dv[address][sgname]

    self.dv = dict(dv)
