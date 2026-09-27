import math
import numbers
from collections import defaultdict, deque
from dataclasses import dataclass, field
from functools import lru_cache

from opendbc.car.carlog import carlog
from opendbc.can.dbc import DBC, Signal


MAX_BAD_COUNTER = 5
CAN_INVALID_CNT = 5


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
  timeout_threshold: float = 1e5  # default to 1Hz threshold
  vals: list[float] = field(default_factory=list)
  all_vals: list[list[float]] = field(default_factory=list)
  timestamps: deque[int] = field(default_factory=lambda: deque(maxlen=500))
  counter: int = 0
  counter_fail: int = 0
  first_seen_nanos: int = 0
  last_warning_log_nanos: int = 0

  def __post_init__(self):
    self.signal_names = [sig.name for sig in self.signals]
    signal_bits = [
      (0 if sig.is_little_endian else 1,
       -1 if sig.msb >= self.size * 8 else sig.lsb if sig.is_little_endian else self.size * 8 - (sig.msb ^ 7) - sig.size,
       (1 << sig.size) - 1, (1 << (sig.size - 1)) if sig.is_signed else 0)
      for sig in self.signals
    ]
    signals, size = self.signals, self.size
    self.check_signals = [(i, sig) for i, sig in enumerate(signals) if sig.calc_checksum is not None or sig.type == 1]

    # Repeated payloads are common. Cache only stateless decoding, never counter,
    # checksum, freshness or validity checks, which must run on every message.
    @lru_cache(maxsize=128)
    def decode(dat):
      data = (int.from_bytes(dat, "little"), int.from_bytes(dat, "big"))
      raw_values = []
      values = []
      for sig, (endian, shift, mask, sign) in zip(signals, signal_bits, strict=True):
        raw = (data[endian] >> shift) & mask if len(dat) == size and shift >= 0 else get_raw_value(dat, sig)
        raw = (raw ^ sign) - sign
        raw_values.append(raw)
        values.append(raw * sig.factor + sig.offset)
      return raw_values, values

    self.decode = decode

  def rate_limited_log(self, last_update_nanos: int, msg: str) -> None:
    if (last_update_nanos - self.last_warning_log_nanos) >= 1_000_000_000:
      carlog.warning(f"CANParser: {hex(self.address)} {self.name} {msg}")
      self.last_warning_log_nanos = last_update_nanos

  def parse(self, nanos: int, dat: bytes) -> bool:
    raw_values, tmp_vals = self.decode(bytes(dat))
    checksum_failed = False
    counter_failed = False

    if self.first_seen_nanos == 0:
      self.first_seen_nanos = nanos

    for i, sig in self.check_signals:
      tmp = raw_values[i]

      if not self.ignore_checksum and sig.calc_checksum is not None:
        expected_checksum = sig.calc_checksum(self.address, sig, bytearray(dat))
        if tmp != expected_checksum:
          checksum_failed = True
          self.rate_limited_log(nanos, f"checksum failed: received {hex(tmp)}, calculated {hex(expected_checksum)}")

      if not self.ignore_counter and sig.type == 1:  # COUNTER
        if not self.update_counter(tmp, sig.size):
          counter_failed = True

    # must have good counter and checksum to update data
    if checksum_failed or counter_failed:
      return False

    if not self.vals:
      self.vals = [0.0] * len(self.signals)
      self.all_vals = [[] for _ in self.signals]

    self.vals[:] = tmp_vals
    for values, value in zip(self.all_vals, tmp_vals, strict=True):
      values.append(value)

    self.timestamps.append(nanos)

    if self.frequency < 1e-5 and len(self.timestamps) >= 3:
      dt = (self.timestamps[-1] - self.timestamps[0]) * 1e-9
      if (dt > 1.0 or (self.timestamps.maxlen is not None and len(self.timestamps) >= self.timestamps.maxlen)) and dt != 0:
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

  def valid(self, current_nanos: int) -> bool:
    if self.ignore_alive:
      return True
    if not self.timestamps:
      return False
    if (current_nanos - self.timestamps[-1]) > self.timeout_threshold:
      return False
    return True


class VLDict(dict):
  def __init__(self, parser):
    super().__init__()
    self.parser = parser

  def __missing__(self, key):
    self.parser._add_message(key)
    return self[key]


class CANParser:
  def __init__(self, dbc_name: str, messages: list[tuple[str | int, int]], bus: int):
    self.dbc_name: str = dbc_name
    self.bus: int = bus
    self.dbc = DBC(dbc_name)

    self.vl: dict[int | str, dict[str, float]] = VLDict(self)
    self.vl_all: dict[int | str, dict[str, list[float]]] = {}
    self.ts_nanos: dict[int | str, dict[str, int]] = {}
    self.addresses: set[int] = set()
    self.message_states: dict[int, MessageState] = {}
    self._updated_addrs: set[int] = set()

    for name_or_addr, freq in messages:
      if isinstance(name_or_addr, numbers.Number):
        msg = self.dbc.addr_to_msg.get(int(name_or_addr))
      else:
        msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        raise RuntimeError(f"could not find message {name_or_addr!r} in DBC {dbc_name}")
      if msg.address in self.addresses:
        raise RuntimeError("Duplicate Message Check: %d" % msg.address)

      self._add_message(name_or_addr, freq)

    self.can_invalid_cnt: int = CAN_INVALID_CNT
    self.last_nonempty_nanos: int = 0
    self._last_update_nanos: int = 0

  def _add_message(self, name_or_addr: str | int, freq: int | None = None) -> None:
    if isinstance(name_or_addr, numbers.Number):
      msg = self.dbc.addr_to_msg.get(int(name_or_addr))
    else:
      msg = self.dbc.name_to_msg.get(name_or_addr)
    assert msg is not None
    assert msg.address not in self.addresses

    self.addresses.add(msg.address)
    signal_names = list(msg.sigs.keys())
    signals_dict = {s: 0.0 for s in signal_names}
    dict.__setitem__(self.vl, msg.address, signals_dict)
    dict.__setitem__(self.vl, msg.name, signals_dict)
    self.vl_all[msg.address] = defaultdict(list)
    self.vl_all[msg.name] = self.vl_all[msg.address]
    self.ts_nanos[msg.address] = {s: 0 for s in signal_names}
    self.ts_nanos[msg.name] = self.ts_nanos[msg.address]

    state = MessageState(
      address=msg.address,
      name=msg.name,
      size=msg.size,
      signals=list(msg.sigs.values()),
      ignore_alive=freq is not None and math.isnan(freq),
    )
    if freq is not None and freq > 0:
      state.frequency = freq
    else:
      # if frequency not specified, assume 1Hz until we learn it
      freq = 1
    state.timeout_threshold = (1_000_000_000 / freq) * 10

    self.message_states[msg.address] = state

  @property
  def bus_timeout(self) -> bool:
    if self._last_update_nanos <= self.last_nonempty_nanos:
      return False
    ignore_alive = all(s.ignore_alive for s in self.message_states.values())
    bus_timeout_threshold = 500 * 1_000_000
    for st in self.message_states.values():
      if st.timeout_threshold > 0:
        bus_timeout_threshold = min(bus_timeout_threshold, st.timeout_threshold)
    return ((self._last_update_nanos - self.last_nonempty_nanos) > bus_timeout_threshold) and not ignore_alive

  @property
  def can_valid(self) -> bool:
    valid = True
    counters_valid = True
    for state in self.message_states.values():
      if state.counter_fail >= MAX_BAD_COUNTER:
        counters_valid = False
        state.rate_limited_log(self._last_update_nanos, f"counter invalid, {state.counter_fail=} {MAX_BAD_COUNTER=}")
      if not state.valid(self._last_update_nanos):
        valid = False
        state.rate_limited_log(self._last_update_nanos, "not valid (timeout or missing)")

    # TODO: probably only want to increment this once per update() call
    self.can_invalid_cnt = 0 if valid else min(self.can_invalid_cnt + 1, CAN_INVALID_CNT)
    return self.can_invalid_cnt < CAN_INVALID_CNT and counters_valid

  def update(self, strings, sendcan: bool = False):
    if strings and not isinstance(strings[0], list | tuple):
      strings = [strings]

    for addr in self._updated_addrs:
      for vals in self.vl_all[addr].values():
        vals.clear()

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

      if not bus_empty:
        self.last_nonempty_nanos = t

      self._last_update_nanos = t

    for address in updated_addrs:
      state = self.message_states[address]
      self.vl[address].update(zip(state.signal_names, state.vals, strict=True))
      self.vl_all[address].update(zip(state.signal_names, state.all_vals, strict=True))
      self.ts_nanos[address].update(dict.fromkeys(state.signal_names, state.timestamps[-1]))

    self._updated_addrs = updated_addrs.copy()
    return updated_addrs


class CANDefine:
  def __init__(self, dbc_name: str):
    dbc = DBC(dbc_name)

    dv = defaultdict(dict)
    for val in dbc.vals:
      sgname = val.name
      address = val.address
      msg = dbc.addr_to_msg.get(address)
      if msg is None:
        raise KeyError(address)
      msgname = msg.name
      parts = val.def_val.split()
      values = [int(v) for v in parts[::2]]
      defs = parts[1::2]
      dv[address][sgname] = dict(zip(values, defs, strict=True))
      dv[msgname][sgname] = dv[address][sgname]

    self.dv = dict(dv)
