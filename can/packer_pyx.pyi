from typing import Dict, List, Tuple, Any
from opendbc.can.common import CANPacker as cpp_CANPacker
from opendbc.can.common import DBC

class CANPacker():
  def __init__(self, dbc_name: str) -> None:
    self.packer: cpp_CANPacker
    self.dbc: DBC
    self.name_to_address_and_size: Dict[str, Tuple[int, int]]
    self.address_to_size: Dict[int, int]
    ...
    
  def pack(self, addr: int, values: Dict[str, Any], counter: int) -> List[int]: ...
  
  def make_can_msg(self, name_or_addr: str|int, bus: int, values: Dict[str, Any], counter: int = ...) -> List[Any] : ...