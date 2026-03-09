import unittest
from unittest.mock import Mock, MagicMock
from opendbc.can.manager import CANIgnitionHooks
from opendbc.can.parser import CANParser
from opendbc.can.types import CanData

class TestCANIgnitionHooks(unittest.TestCase):
  
  def setUp(self):
    # Create a mock CAN parser
    self.parser = Mock(spec=CANParser)
    self.parser.msgs = {}
    
    # Create ignition hooks
    self.ignition_hooks = CANIgnitionHooks(self.parser, ['ignition_signal'])
    
    # Mock CAN data
    self.can_data = []
  
  def test_initial_state(self):
    """Test initial ignition state is False"""
    self.assertFalse(self.ignition_hooks.is_ignition_on())
  
  def test_ignition_on(self):
    """Test detection of ignition on"""
    # Set up parser to return ignition signal active
    self.parser.msgs = {'ignition_signal': {'value': 12}}
    
    # Update with CAN data
    changed = self.ignition_hooks.update(self.can_data)
    
    # State should have changed
    self.assertTrue(changed)
    self.assertTrue(self.ignition_hooks.is_ignition_on())
  
  def test_ignition_off(self):
    """Test detection of ignition off"""
    # First turn ignition on
    self.parser.msgs = {'ignition_signal': {'value': 12}}
    self.ignition_hooks.update(self.can_data)
    self.assertTrue(self.ignition_hooks.is_ignition_on())
    
    # Then turn ignition off
    self.parser.msgs = {'ignition_signal': {'value': 0}}
    changed = self.ignition_hooks.update(self.can_data)
    
    # State should have changed
    self.assertTrue(changed)
    self.assertFalse(self.ignition_hooks.is_ignition_on())
  
  def test_callback(self):
    """Test callback functionality"""
    callback = Mock()
    self.ignition_hooks.add_callback(callback)
    
    # Trigger ignition on
    self.parser.msgs = {'ignition_signal': {'value': 12}}
    self.ignition_hooks.update(self.can_data)
    
    # Callback should have been called with True
    callback.assert_called_with(True)
    
    # Reset callback
    callback.reset_mock()
    
    # Trigger ignition off
    self.parser.msgs = {'ignition_signal': {'value': 0}}
    self.ignition_hooks.update(self.can_data)
    
    # Callback should have been called with False
    callback.assert_called_with(False)
  
  def test_remove_callback(self):
    """Test removing callbacks"""
    callback = Mock()
    self.ignition_hooks.add_callback(callback)
    self.ignition_hooks.remove_callback(callback)
    
    # Trigger ignition on
    self.parser.msgs = {'ignition_signal': {'value': 12}}
    self.ignition_hooks.update(self.can_data)
    
    # Callback should not have been called
    callback.assert_not_called()
  
  def test_throttle_rate_limiting(self):
    """Test that updates are throttled to at most once per second"""
    # Set up parser to return ignition signal active
    self.parser.msgs = {'ignition_signal': {'value': 12}}
    
    # First update should change state
    changed = self.ignition_hooks.update(self.can_data)
    self.assertTrue(changed)
    
    # Second update immediately should not change state
    changed = self.ignition_hooks.update(self.can_data)
    self.assertFalse(changed)
    
    # Update after waiting more than a second should change state
    import time
    time.sleep(1.1)
    changed = self.ignition_hooks.update(self.can_data)
    self.assertTrue(changed)
  
  def test_multiple_signals(self):
    """Test with multiple ignition signals"""
    # Update hooks with multiple signals
    self.ignition_hooks = CANIgnitionHooks(self.parser, ['ignition_1', 'ignition_2'])
    
    # First signal active
    self.parser.msgs = {'ignition_1': {'value': 12}}
    changed = self.ignition_hooks.update(self.can_data)
    self.assertTrue(changed)
    self.assertTrue(self.ignition_hooks.is_ignition_on())
    
    # Reset
    self.parser.msgs = {}
    self.ignition_hooks = CANIgnitionHooks(self.parser, ['ignition_1', 'ignition_2'])
    
    # Second signal active
    self.parser.msgs = {'ignition_2': {'value': 12}}
    changed = self.ignition_hooks.update(self.can_data)
    self.assertTrue(changed)
    self.assertTrue(self.ignition_hooks.is_ignition_on())
  
  def test_error_handling(self):
    """Test that errors in callbacks don't break the system"""
    def bad_callback(state):
      raise Exception("Callback error")
    
    good_callback = Mock()
    self.ignition_hooks.add_callback(bad_callback)
    self.ignition_hooks.add_callback(good_callback)
    
    # Trigger ignition on
    self.parser.msgs = {'ignition_signal': {'value': 12}}
    self.ignition_hooks.update(self.can_data)
    
    # Good callback should still be called
    good_callback.assert_called_with(True)
    
    # State should still be updated
    self.assertTrue(self.ignition_hooks.is_ignition_on())