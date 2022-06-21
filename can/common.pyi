from dataclasses import dataclass

from enum import Enum
from typing import Any, List

class SignalType(Enum):
  DEFAULT = 0
  HONDA_CHECKSUM = 1
  HONDA_COUNTER = 2
  TOYOTA_CHECKSUM = 3
  PEDAL_CHECKSUM = 4
  PEDAL_COUNTER = 5
  VOLKSWAGEN_CHECKSUM = 6
  VOLKSWAGEN_COUNTER = 7
  SUBARU_CHECKSUM = 8
  CHRYSLER_CHECKSUM = 9
  HKG_CAN_FD_CHECKSUM = 10
  HKG_CAN_FD_COUNTER = 11

@dataclass
class Signal():
  name: str
  start_bit: int
  msb: int
  lsb: int
  size: int
  is_signed: bool
  factor: float
  offset: float
  is_little_endian: bool
  type: SignalType

@dataclass
class Msg():
  name: str
  address: int
  size: int
  sigs: List[Signal]

@dataclass
class Val():
  name: str
  address: int
  def_val: str
  sigs: List[Signal]

@dataclass
class DBC():
  name: str
  msgs: List[Msg]
  vals: List[Val]

@dataclass
class SignalParseOptions():
  address: int
  name: str


@dataclass
class MessageParseOptions():
  address: int
  check_frequency: int

@dataclass
class SignalValue():
  address: int
  name: str
  value: Any # TODO: c++ double... - maybe debug to get runtime type
  all_values: List[Any]

@dataclass
class SignalPackValue():
    name: str
    value: Any # TODO: c++ double... - maybe debug to get runtime type

def dbc_lookup(dbc_name: str) -> DBC: ...
