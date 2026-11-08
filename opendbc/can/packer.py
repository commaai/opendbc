import math

from opendbc.car.carlog import carlog
from opendbc.can.dbc import DBC, Signal


class CANPacker:
  def __init__(self, dbc_name: str):
    self.dbc = DBC(dbc_name)
    self.counters: dict[int, int] = {}

  def pack(self, address: int, values: dict[str, float]) -> bytearray:
    msg = self.dbc.addr_to_msg.get(address)
    if msg is None:
      carlog.error(f"msg not found for {address=}")
      return bytearray()
    dat = bytearray(msg.size)
    counter_set = False
    for name, value in values.items():
      sig = msg.sigs.get(name)
      if sig is None:
        carlog.error(f"unknown signal {name=} in {msg.name}")
        continue
      ival = int(math.floor((value - sig.offset) / sig.factor + 0.5))
      if ival < 0:
        ival = (1 << sig.size) + ival
      set_value(dat, sig, ival)
      if sig is msg.counter_sig:
        self.counters[address] = int(value)
        counter_set = True
    if msg.counter_sig and not counter_set:
      counter = self.counters.get(address, 0)
      set_value(dat, msg.counter_sig, counter)
      self.counters[address] = (counter + 1) % (1 << msg.counter_sig.size)
    if msg.checksum_sig and msg.checksum_sig.calc_checksum:
      checksum = msg.checksum_sig.calc_checksum(address, msg.checksum_sig, dat)
      set_value(dat, msg.checksum_sig, checksum)
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
  if sig.size < 64:
    ival &= (1 << sig.size) - 1
  for i, shift, size, mask in sig.segments:
    msg[i] = (msg[i] & ~mask) | ((ival << shift) & mask)
    ival >>= size
