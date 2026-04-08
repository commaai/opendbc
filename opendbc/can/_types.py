from typing import Any, Final

from mypy_extensions import mypyc_attr


class SignalType:
  DEFAULT: Final = 0
  COUNTER: Final = 1
  HONDA_CHECKSUM: Final = 2
  TOYOTA_CHECKSUM: Final = 3
  BODY_CHECKSUM: Final = 4
  VOLKSWAGEN_MQB_MEB_CHECKSUM: Final = 5
  XOR_CHECKSUM: Final = 6
  SUBARU_CHECKSUM: Final = 7
  CHRYSLER_CHECKSUM: Final = 8
  HKG_CAN_FD_CHECKSUM: Final = 9
  FCA_GIORGIO_CHECKSUM: Final = 10
  TESLA_CHECKSUM: Final = 11
  PSA_CHECKSUM: Final = 12
  VOLKSWAGEN_MLB_CHECKSUM: Final = 13


@mypyc_attr(allow_interpreted_subclasses=True)
class Signal:
  __slots__ = ('name', 'start_bit', 'msb', 'lsb', 'size', 'is_signed',
               'factor', 'offset', 'is_little_endian', 'type', 'calc_checksum')

  name: str
  start_bit: int
  msb: int
  lsb: int
  size: int
  is_signed: bool
  factor: float
  offset: float
  is_little_endian: bool
  type: int
  calc_checksum: Any  # Callable[[int, Signal, bytearray], int] | None

  def __init__(
    self,
    name: str,
    start_bit: int,
    msb: int,
    lsb: int,
    size: int,
    is_signed: bool,
    factor: float,
    offset: float,
    is_little_endian: bool,
    type: int = 0,  # noqa: A002
    calc_checksum: Any = None,
  ) -> None:
    self.name = name
    self.start_bit = start_bit
    self.msb = msb
    self.lsb = lsb
    self.size = size
    self.is_signed = is_signed
    self.factor = factor
    self.offset = offset
    self.is_little_endian = is_little_endian
    self.type = type
    self.calc_checksum = calc_checksum


class Msg:  # noqa: B903
  __slots__ = ('name', 'address', 'size', 'sigs')

  name: str
  address: int
  size: int
  sigs: dict[str, Signal]

  def __init__(self, name: str, address: int, size: int, sigs: dict[str, Signal]) -> None:
    self.name = name
    self.address = address
    self.size = size
    self.sigs = sigs


class Val:  # noqa: B903
  __slots__ = ('name', 'address', 'def_val', 'sigs')

  name: str
  address: int
  def_val: str
  sigs: Any  # dict[str, Signal] | None

  def __init__(self, name: str, address: int, def_val: str, sigs: Any = None) -> None:
    self.name = name
    self.address = address
    self.def_val = def_val
    self.sigs = sigs
