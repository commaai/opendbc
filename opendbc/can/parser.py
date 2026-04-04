import math
from collections import defaultdict, deque
from typing import Any

from opendbc.car.carlog import carlog
from opendbc.can._types import Signal, SignalType
from opendbc.can._vldict import VLDict


MAX_BAD_COUNTER: int = 5
CAN_INVALID_CNT: int = 5


def get_raw_value(dat: bytes, sig: Signal) -> int:
  sig_lsb: int = sig.lsb
  sig_msb: int = sig.msb
  sig_le: bool = sig.is_little_endian
  ret: int = 0
  i: int = sig_msb // 8
  bits: int = sig.size
  dat_len: int = len(dat)
  while 0 <= i < dat_len and bits > 0:
    lsb: int = sig_lsb if (sig_lsb // 8) == i else i * 8
    msb: int = sig_msb if (sig_msb // 8) == i else (i + 1) * 8 - 1
    size: int = msb - lsb + 1
    d: int = (dat[i] >> (lsb - (i * 8))) & ((1 << size) - 1)
    ret |= d << (bits - size)
    bits -= size
    i = i - 1 if sig_le else i + 1
  return ret


class MessageState:
  __slots__ = ('address', 'name', 'size', 'signals', 'ignore_alive',
               'ignore_checksum', 'ignore_counter', 'frequency',
               'timeout_threshold', 'vals', 'all_vals', 'timestamps',
               'counter', 'counter_fail', 'first_seen_nanos',
               'last_warning_log_nanos')

  address: int
  name: str
  size: int
  signals: list[Signal]
  ignore_alive: bool
  ignore_checksum: bool
  ignore_counter: bool
  frequency: float
  timeout_threshold: float
  vals: list[float]
  all_vals: list[list[float]]
  timestamps: deque[int]
  counter: int
  counter_fail: int
  first_seen_nanos: int
  last_warning_log_nanos: int

  def __init__(
    self,
    address: int,
    name: str,
    size: int,
    signals: list[Signal],
    ignore_alive: bool = False,
    ignore_checksum: bool = False,
    ignore_counter: bool = False,
    frequency: float = 0.0,
    timeout_threshold: float = 1e5,
  ) -> None:
    self.address = address
    self.name = name
    self.size = size
    self.signals = signals
    self.ignore_alive = ignore_alive
    self.ignore_checksum = ignore_checksum
    self.ignore_counter = ignore_counter
    self.frequency = frequency
    self.timeout_threshold = timeout_threshold
    self.vals: list[float] = []
    self.all_vals: list[list[float]] = []
    self.timestamps: deque[int] = deque(maxlen=500)
    self.counter: int = 0
    self.counter_fail: int = 0
    self.first_seen_nanos: int = 0
    self.last_warning_log_nanos: int = 0

  def rate_limited_log(self, last_update_nanos: int, msg: str) -> None:
    if (last_update_nanos - self.last_warning_log_nanos) >= 1_000_000_000:
      carlog.warning(f"CANParser: {hex(self.address)} {self.name} {msg}")
      self.last_warning_log_nanos = last_update_nanos

  def parse(self, nanos: int, dat: bytes) -> bool:
    num_signals: int = len(self.signals)
    tmp_vals: list[float] = [0.0] * num_signals
    checksum_failed: bool = False
    counter_failed: bool = False

    if self.first_seen_nanos == 0:
      self.first_seen_nanos = nanos

    for i in range(num_signals):
      sig: Signal = self.signals[i]
      tmp: int = get_raw_value(dat, sig)
      if sig.is_signed:
        tmp -= ((tmp >> (sig.size - 1)) & 0x1) * (1 << sig.size)

      if not self.ignore_checksum and sig.calc_checksum is not None:
        expected_checksum: int = sig.calc_checksum(self.address, sig, bytearray(dat))
        if tmp != expected_checksum:
          checksum_failed = True
          self.rate_limited_log(nanos, f"checksum failed: received {hex(tmp)}, calculated {hex(expected_checksum)}")

      if not self.ignore_counter and sig.type == SignalType.COUNTER:
        if not self.update_counter(tmp, sig.size):
          counter_failed = True

      tmp_vals[i] = tmp * sig.factor + sig.offset

    # must have good counter and checksum to update data
    if checksum_failed or counter_failed:
      return False

    if not self.vals:
      self.vals = [0.0] * num_signals
      self.all_vals = [[] for _ in range(num_signals)]

    for i in range(num_signals):
      v: float = tmp_vals[i]
      self.vals[i] = v
      self.all_vals[i].append(v)

    self.timestamps.append(nanos)

    if self.frequency < 1e-5 and len(self.timestamps) >= 3:
      dt: float = (self.timestamps[-1] - self.timestamps[0]) * 1e-9
      ts_maxlen = self.timestamps.maxlen
      if (dt > 1.0 or (ts_maxlen is not None and len(self.timestamps) >= ts_maxlen)) and dt != 0:
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
    if (current_nanos - self.timestamps[-1]) > self.timeout_threshold:
      return False
    return True


class CANParser:
  def __init__(self, dbc_name: str, messages: list, bus: int) -> None:
    from opendbc.can.dbc import DBC
    self.dbc_name: str = dbc_name
    self.bus: int = bus
    self.dbc = DBC(dbc_name)

    self.vl: Any = VLDict(self)
    self.vl_all: dict[int | str, dict[str, list[float]]] = {}
    self.ts_nanos: dict[int | str, dict[str, int]] = {}
    self.addresses: set[int] = set()
    self.message_states: dict[int, MessageState] = {}
    self._vl_fast: dict[int, dict[str, float]] = {}

    for m in messages:
      name_or_addr = m[0]
      freq = m[1]
      if isinstance(name_or_addr, int):
        msg = self.dbc.addr_to_msg.get(name_or_addr)
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

  def _add_message(self, name_or_addr: str | int, freq: float | int | None = None) -> None:
    if isinstance(name_or_addr, int):
      msg = self.dbc.addr_to_msg.get(name_or_addr)
    else:
      msg = self.dbc.name_to_msg.get(name_or_addr)
    assert msg is not None
    assert msg.address not in self.addresses

    self.addresses.add(msg.address)
    signal_names: list[str] = list(msg.sigs.keys())
    signals_dict: dict[str, float] = {s: 0.0 for s in signal_names}
    dict.__setitem__(self.vl, msg.address, signals_dict)
    dict.__setitem__(self.vl, msg.name, signals_dict)
    self._vl_fast[msg.address] = signals_dict
    self.vl_all[msg.address] = defaultdict(list)
    self.vl_all[msg.name] = self.vl_all[msg.address]
    self.ts_nanos[msg.address] = {s: 0 for s in signal_names}
    self.ts_nanos[msg.name] = self.ts_nanos[msg.address]

    state: MessageState = MessageState(
      address=msg.address,
      name=msg.name,
      size=msg.size,
      signals=list(msg.sigs.values()),
      ignore_alive=freq is not None and math.isnan(freq),
    )
    if freq is not None and freq > 0:
      state.frequency = float(freq)
    else:
      # if frequency not specified, assume 1Hz until we learn it
      freq = 1
    state.timeout_threshold = (1_000_000_000.0 / freq) * 10.0

    self.message_states[msg.address] = state

  @property
  def bus_timeout(self) -> bool:
    ignore_alive: bool = all(s.ignore_alive for s in self.message_states.values())
    bus_timeout_threshold: float = 500.0 * 1_000_000.0
    for st in self.message_states.values():
      if st.timeout_threshold > 0:
        bus_timeout_threshold = min(bus_timeout_threshold, st.timeout_threshold)
    return ((self._last_update_nanos - self.last_nonempty_nanos) > bus_timeout_threshold) and not ignore_alive

  @property
  def can_valid(self) -> bool:
    valid: bool = True
    counters_valid: bool = True
    bus_timeout: bool = self.bus_timeout
    for state in self.message_states.values():
      if state.counter_fail >= MAX_BAD_COUNTER:
        counters_valid = False
        state.rate_limited_log(self._last_update_nanos, f"counter invalid, {state.counter_fail=} {MAX_BAD_COUNTER=}")
      if not state.valid(self._last_update_nanos, bus_timeout):
        valid = False
        state.rate_limited_log(self._last_update_nanos, "not valid (timeout or missing)")

    # TODO: probably only want to increment this once per update() call
    self.can_invalid_cnt = 0 if valid else min(self.can_invalid_cnt + 1, CAN_INVALID_CNT)
    return self.can_invalid_cnt < CAN_INVALID_CNT and counters_valid

  def update(self, strings: list, sendcan: bool = False) -> set[int]:
    if strings and not isinstance(strings[0], (list, tuple)):
      strings = [strings]

    for addr in self.addresses:
      vl_all_a = self.vl_all[addr]
      for k in vl_all_a:
        vl_all_a[k].clear()

    updated_addrs: set[int] = set()
    for entry in strings:
      frames = entry[1]
      t = int(entry[0])
      bus_empty: bool = True
      for frame in frames:
        address = int(frame[0])
        dat = frame[1]
        src = int(frame[2])
        if src != self.bus:
          continue
        bus_empty = False
        state = self.message_states.get(address)
        if state is None or len(dat) > 64:
          continue
        if state.parse(t, dat):
          updated_addrs.add(address)

          vl_addr = self._vl_fast[address]
          vl_all_addr = self.vl_all[address]
          ts_addr = self.ts_nanos[address]

          signals: list[Signal] = state.signals
          vals: list[float] = state.vals
          last_ts: int = state.timestamps[-1]
          num_signals: int = len(signals)
          for i in range(num_signals):
            sig_name: str = signals[i].name
            vl_addr[sig_name] = vals[i]
            vl_all_addr[sig_name] = state.all_vals[i]
            ts_addr[sig_name] = last_ts

      if not bus_empty:
        self.last_nonempty_nanos = t

      self._last_update_nanos = t

    return updated_addrs


class CANDefine:
  def __init__(self, dbc_name: str) -> None:
    from opendbc.can.dbc import DBC
    dbc = DBC(dbc_name)

    dv: dict[int | str, dict[str, dict[int, str]]] = defaultdict(dict)
    for val in dbc.vals:
      sgname: str = val.name
      address: int = val.address
      msg = dbc.addr_to_msg.get(address)
      if msg is None:
        raise KeyError(address)
      msgname: str = msg.name
      parts: list[str] = val.def_val.split()
      values: list[int] = [int(v) for v in parts[::2]]
      defs: list[str] = parts[1::2]
      dv[address][sgname] = dict(zip(values, defs, strict=True))
      dv[msgname][sgname] = dv[address][sgname]

    self.dv: dict[int | str, dict[str, dict[int, str]]] = dict(dv)
