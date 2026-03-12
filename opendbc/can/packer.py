import math

from mypy_extensions import mypyc_attr

from opendbc.car.carlog import carlog
from opendbc.can._types import Signal, SignalType


def set_value(msg: bytearray, sig: Signal, ival: int) -> None:
  sig_lsb: int = sig.lsb
  sig_size: int = sig.size
  sig_le: bool = sig.is_little_endian
  i: int = sig_lsb // 8
  bits: int = sig_size
  if sig_size < 64:
    ival &= (1 << sig_size) - 1
  msg_len: int = len(msg)
  while 0 <= i < msg_len and bits > 0:
    shift: int = sig_lsb % 8 if (sig_lsb // 8) == i else 0
    size: int = min(bits, 8 - shift)
    mask: int = ((1 << size) - 1) << shift
    msg[i] &= ~mask
    msg[i] |= (ival & ((1 << size) - 1)) << shift
    bits -= size
    ival >>= size
    i = i + 1 if sig_le else i - 1


@mypyc_attr(allow_interpreted_subclasses=True)
class CANPacker:
  def __init__(self, dbc_name: str) -> None:
    from opendbc.can.dbc import DBC
    self.dbc = DBC(dbc_name)
    self.counters: dict[int, int] = {}

  def pack(self, address: int, values: dict[str, float]) -> bytearray:
    msg = self.dbc.addr_to_msg.get(address)
    if msg is None:
      carlog.error(f"msg not found for {address=}")
      return bytearray()
    dat: bytearray = bytearray(msg.size)
    counter_set: bool = False
    for name, value in values.items():
      sig = msg.sigs.get(name)
      if sig is None:
        carlog.error(f"unknown signal {name=} in {msg.name}")
        continue
      ival: int = int(math.floor((value - sig.offset) / sig.factor + 0.5))
      if ival < 0:
        ival = (1 << sig.size) + ival
      set_value(dat, sig, ival)
      if sig.type == SignalType.COUNTER or sig.name == "COUNTER":
        self.counters[address] = int(value)
        counter_set = True
    sig_counter: Signal | None = next((s for s in msg.sigs.values() if s.type == SignalType.COUNTER or s.name == "COUNTER"), None)
    if sig_counter is not None and not counter_set:
      if address not in self.counters:
        self.counters[address] = 0
      set_value(dat, sig_counter, self.counters[address])
      self.counters[address] = (self.counters[address] + 1) % (1 << sig_counter.size)
    sig_checksum: Signal | None = next((s for s in msg.sigs.values() if s.type > SignalType.COUNTER), None)
    if sig_checksum is not None and sig_checksum.calc_checksum is not None:
      checksum: int = sig_checksum.calc_checksum(address, sig_checksum, dat)
      set_value(dat, sig_checksum, checksum)
    return dat

  def make_can_msg(self, name_or_addr: str | int, bus: int, values: dict[str, float]) -> tuple[int, bytes, int]:
    if isinstance(name_or_addr, int):
      addr: int = name_or_addr
    else:
      msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        carlog.error(f"msg not found for {name_or_addr=}")
        return 0, b'', bus
      addr = msg.address
    dat: bytearray = self.pack(addr, values)
    if len(dat) == 0:
      return 0, b'', bus
    return addr, bytes(dat), bus
