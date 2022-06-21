import numbers

from cereal import log
from dataclasses import dataclass
from typing import Any, DefaultDict, Dict, List, Set, Tuple

from opendbc.can.common import *


class CANParser():
  def __init__(self, dbc_name: str, signals: List[Tuple[str|int, str]],
               checks: List[Tuple[str, int]], bus: int = 0, enforce_checks: bool = True) -> None:
    self.can_valid: bool
    self.bus_timeout: bool
    self.first_sec: int
    self.last_sec: int
    self.last_nonempty_sec: int
    self.bus_timeout_threshold: int
    self.dbc_name: str
    self.dbc: DBC
    self.vl: Dict[int|str, Dict[str, Any]]
    """ Get Latest value for a signal. vl["messagename"|messageid]["signalName"] -> signal value""" 
    self.vl_all: Dict[int|str, Dict[str, List[Any]]]
    self.can_invalid_cnt: int
    ...

  def update_string(self, data: str, sendcan: bool) -> None: ...

  def UpdateValid(self, sec: int) -> None: ...
  
  def query_latest(self) -> List[SignalValue]: ...
  
  def update_vl(self) -> Set[int]: ...

  # #ifndef DYNAMIC_CAPNP
  # void UpdateCans(uint64_t sec, const capnp::List<cereal::CanData>::Reader& cans);
  # #endif
  # void UpdateCans(uint64_t sec, const capnp::DynamicStruct::Reader& cans);

class CANDefine():
  """Helper class to convert raw int signal values to their mapped names"""
  def __init__(self, dbc_name: str) -> None:
    self.dbc_name: str
    self.dbc: DBC
    self.dv: Dict[str|int, Dict[str, Dict[int, str]]]
    """Example: dv["Message"]["signal"][1] -> "LEFT" """
