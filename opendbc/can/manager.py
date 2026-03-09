from opendbc.can.parser import CANParser
from opendbc.can.types import CanData
from typing import Optional, Dict, List, Tuple
import time

class CANIgnitionHooks:
  """Handles CAN ignition detection and hooks"""
  
  def __init__(self, can_parser: CANParser, ignition_signals: List[str]):
    self.can_parser = can_parser
    self.ignition_signals = ignition_signals
    self.ignition_state = False
    self.last_update_time = 0
    self.callbacks = []
  
  def update(self, can_data: List[CanData]) -> bool:
    """Update ignition state based on CAN data. Returns True if state changed."""
    current_time = time.time()
    
    # Only check at most once per second
    if current_time - self.last_update_time < 1.0:
      return False
    
    self.last_update_time = current_time
    
    # Check if any ignition signal is active
    new_ignition_state = False
    for signal in self.ignition_signals:
      if signal in self.can_parser.msgs and self.can_parser.msgs[signal]['value'] > 0:
        new_ignition_state = True
        break
    
    if new_ignition_state != self.ignition_state:
      self.ignition_state = new_ignition_state
      self._run_callbacks()
      return True
    
    return False
  
  def add_callback(self, callback):
    """Add a callback to be called when ignition state changes"""
    self.callbacks.append(callback)
  
  def remove_callback(self, callback):
    """Remove a callback"""
    if callback in self.callbacks:
      self.callbacks.remove(callback)
  
  def _run_callbacks(self):
    """Run all registered callbacks"""
    for callback in self.callbacks:
      try:
        callback(self.ignition_state)
      except Exception as e:
        print(f"Error in ignition callback: {e}")
  
  def is_ignition_on(self) -> bool:
    """Return current ignition state"""
    return self.ignition_state