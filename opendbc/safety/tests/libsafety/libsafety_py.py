import os
from typing import Protocol, Any

from opendbc.safety import LEN_TO_DLC
from opendbc.safety.tests.libsafety.safety_helpers import PandaSafety, setup_safety_helpers

libsafety_dir = os.path.dirname(os.path.abspath(__file__))
libsafety_fn = os.path.join(libsafety_dir, "libsafety.so")

_CDEFS: list[tuple[str, dict]] = [
  ("""
typedef struct {
  unsigned char fd : 1;
  unsigned char bus : 3;
  unsigned char data_len_code : 4;
  unsigned char rejected : 1;
  unsigned char returned : 1;
  unsigned char extended : 1;
  unsigned int addr : 29;
  unsigned char checksum;
  unsigned char data[64];
} CANPacket_t;
""", {"packed": True}),
  ("""
bool safety_rx_hook(CANPacket_t *msg);
bool safety_tx_hook(CANPacket_t *msg);
int safety_fwd_hook(int bus_num, int addr);
int set_safety_hooks(uint16_t mode, uint16_t param);
""", {}),
  ("""
void can_set_checksum(CANPacket_t *packet);
""", {}),
]


# lazy ffi proxy
class _LazyFFI:
  def __init__(self):
    self._real = None

  def _ensure(self):
    if self._real is None:
      from cffi import FFI
      real = FFI()
      for src, kwargs in _CDEFS:
        if kwargs:
          real.cdef(src, **kwargs)
        else:
          real.cdef(src)
      setup_safety_helpers(real)
      self._real = real
    return self._real

  def __getattr__(self, name: str) -> Any:
    return getattr(self._ensure(), name)

  def __repr__(self) -> str:
    return "<LazyFFI (loaded)>" if self._real else "<LazyFFI (not loaded)>"


ffi = _LazyFFI()


# lazy libsafety proxy
class _LazyLib:
  def __init__(self, path: str):
    self._path = path
    self._real = None

  def _ensure(self):
    if self._real is None:
      real_ffi = ffi._ensure()
      self._real = real_ffi.dlopen(self._path)
    return self._real

  def __getattr__(self, name: str) -> Any:
    return getattr(self._ensure(), name)

  def __repr__(self) -> str:
    return f"<LazyLib {os.path.basename(self._path)} (loaded)>" if self._real else f"<LazyLib {os.path.basename(self._path)} (not loaded)>"


class CANPacket:
  reserved: int
  bus: int
  data_len_code: int
  rejected: int
  returned: int
  extended: int
  addr: int
  data: list[int]


class Panda(PandaSafety, Protocol):
  # CAN
  def can_set_checksum(self, p: CANPacket) -> None: ...

  # safety
  def safety_rx_hook(self, msg: CANPacket) -> int: ...
  def safety_tx_hook(self, msg: CANPacket) -> int: ...
  def safety_fwd_hook(self, bus_num: int, addr: int) -> int: ...
  def set_safety_hooks(self, mode: int, param: int) -> int: ...


libsafety: Panda = _LazyLib(libsafety_fn)

# helpers


def make_CANPacket(addr: int, bus: int, dat):
  ret = ffi.new('CANPacket_t *')
  ret[0].extended = 1 if addr >= 0x800 else 0
  ret[0].addr = addr
  ret[0].data_len_code = LEN_TO_DLC[len(dat)]
  ret[0].bus = bus
  ret[0].data = bytes(dat)
  libsafety.can_set_checksum(ret)

  return ret
