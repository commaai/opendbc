from typing import Dict, List, Set, Tuple, Optional
from opendbc.can.common import CANParser as cpp_CANParser
from opendbc.can.common import SignalValue, DBC


class CANParser():
  def __init__(self, dbc_name: str, signals: List[Tuple[str|int, str]],
               checks: Optional[List[Tuple[str, int]]] = None, bus: Optional[int] = 0, enforce_checks: Optional[bool] = True) -> None:
    self.can: cpp_CANParser
    self.dbc: DBC
    self.msg_name_to_address: Dict[str,int]
    self.address_to_msg_name: Dict[int,str]
    self.can_values: List[SignalValue]
    # readonly
    self.vl: Dict[int|str, Dict[str, float]]
    """ Get Latest value for a signal. vl["messagename"|messageid]["signalName"] -> signal value""" 
    self.vl_all: Dict[int|str, Dict[str, List[float]]]
    self.can_valid: bool
    self.bus_timeout: bool
    self.dbc_name: str
    self.can_invalid_cnt: int
    ...

  def update_vl(self) -> Set[int]: ...

  def update_string(self, dat: str, sendcan: Optional[bool] = False) -> Set[int]: ...

  def update_strings(self, strings: List[str], sendcan: Optional[bool] = False) -> Set[int]: ...

  def UpdateValid(self, sec: int) -> None: ...
  
  def query_latest(self) -> List[SignalValue]: ...
  

class CANDefine():
  def __init__(self, dbc_name: str) -> None:
    self.dbc: DBC
    self.dv: Dict[str|int, Dict[str, Dict[int, str]]]
    """Example: dv["Message"]["signal"][1] -> "LEFT" """
    self.dbc_name: str
    ...
