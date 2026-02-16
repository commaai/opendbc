import math

from opendbc.car.carlog import carlog
from opendbc.can.dbc import DBC, Signal, SignalType

_floor = math.floor


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
    sigs = msg.sigs
    floor = _floor
    for name, value in values.items():
      sig = sigs.get(name)
      if sig is None:
        carlog.error(f"unknown signal {name=} in {msg.name}")
        continue
      ival = int(floor((value - sig.offset) / sig.factor + 0.5))
      if ival < 0:
        ival += 1 << sig.size
      # Inlined optimized set_value using pre-computed ops
      ival &= sig._size_mask
      for byte_idx, clear_mask, shift, val_mask, n_bits in sig._pack_ops:
        dat[byte_idx] = (dat[byte_idx] & clear_mask) | ((ival & val_mask) << shift)
        ival >>= n_bits
      if sig.is_counter:
        self.counters[address] = int(value)
        counter_set = True
    sig_counter = msg.sig_counter
    if sig_counter is not None and not counter_set:
      counter = self.counters.get(address, 0)
      ival = counter & sig_counter._size_mask
      for byte_idx, clear_mask, shift, val_mask, n_bits in sig_counter._pack_ops:
        dat[byte_idx] = (dat[byte_idx] & clear_mask) | ((ival & val_mask) << shift)
        ival >>= n_bits
      self.counters[address] = (counter + 1) % (1 << sig_counter.size)
    sig_checksum = msg.sig_checksum
    if sig_checksum is not None and sig_checksum.calc_checksum is not None:
      checksum = sig_checksum.calc_checksum(address, sig_checksum, dat)
      ival = checksum & sig_checksum._size_mask
      for byte_idx, clear_mask, shift, val_mask, n_bits in sig_checksum._pack_ops:
        dat[byte_idx] = (dat[byte_idx] & clear_mask) | ((ival & val_mask) << shift)
        ival >>= n_bits
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
  ival &= sig._size_mask
  for byte_idx, clear_mask, shift, val_mask, n_bits in sig._pack_ops:
    msg[byte_idx] = (msg[byte_idx] & clear_mask) | ((ival & val_mask) << shift)
    ival >>= n_bits
