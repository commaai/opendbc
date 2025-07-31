import math

from opendbc.car.carlog import carlog
from opendbc.can.dbc import DBC, Signal, SignalType


class CANPacker:
  def __init__(self, dbc_name: str):
    self.dbc = DBC(dbc_name)
    self.counters: dict[int, int] = {}

    self.counter_signals: dict[int, Signal] = {}
    self.checksum_signals: dict[int, Signal] = {}

    for addr, msg in self.dbc.addr_to_msg.items():
      for sig in msg.sigs.values():
        if sig.type == SignalType.COUNTER or sig.name == "COUNTER":
          self.counter_signals[addr] = sig
        elif sig.type > SignalType.COUNTER and sig.calc_checksum:
          self.checksum_signals[addr] = sig

  def pack(self, address: int, values: dict[str, float]) -> bytearray:
    msg = self.dbc.addr_to_msg.get(address)
    if msg is None:
      carlog.error(f"msg not found for {address=}")
      return bytearray()
    dat = bytearray(msg.size)
    counter_set = False
    sig_counter = self.counter_signals.get(address)
    sig_checksum = self.checksum_signals.get(address)
    for name, value in values.items():
      sig = msg.sigs.get(name)
      if sig is None:
        carlog.error(f"unknown signal {name=} in {msg.name}")
        continue
      ival = int(math.floor((value - sig.offset) / sig.factor + 0.5))
      if ival < 0:
        ival = (1 << sig.size) + ival
      set_value(dat, sig, ival)
      if sig is sig_counter:
        self.counters[address] = int(value)
        counter_set = True

    if sig_counter and not counter_set:
      if address not in self.counters:
        self.counters[address] = 0
      set_value(dat, sig_counter, self.counters[address])
      self.counters[address] = (self.counters[address] + 1) % (1 << sig_counter.size)

    if sig_checksum:
      checksum = sig_checksum.calc_checksum(address, sig_checksum, dat)
      set_value(dat, sig_checksum, checksum)
    return dat

  def make_can_msg(self, name_or_addr, bus: int, values: dict[str, float]):
    if isinstance(name_or_addr, int):
      addr = name_or_addr
    else:
      msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        carlog.error(f"msg not found for {name_or_addr=}")
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
