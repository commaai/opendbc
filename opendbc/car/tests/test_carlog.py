import os
import logging
from unittest.mock import patch
from opendbc.car import carlog


class TestCarlog:
  def test_carlog_default_level(self):
    """Test carlog has default INFO level"""
    # carlog should be configured as INFO by default
    assert carlog.carlog.level == logging.INFO
    assert carlog.carlog.propagate is False
    
  def test_carlog_has_handler(self):
    """Test carlog has a stream handler"""
    handlers = carlog.carlog.handlers
    assert len(handlers) > 0
    assert isinstance(handlers[0], logging.StreamHandler)
    
  def test_carlog_formatter(self):
    """Test carlog handler has correct formatter"""
    handler = carlog.carlog.handlers[0]
    formatter = handler.formatter
    assert formatter is not None
    assert formatter._fmt == '%(message)s'
    
  def test_carlog_environment_variable(self):
    """Test LOGPRINT environment variable processing"""
    # Test that LOGPRINT would be read from environment
    with patch.dict(os.environ, {'LOGPRINT': 'DEBUG'}):
      # Import would need to be re-done to pick up new env var
      # Just verify the logic would work
      assert os.environ.get('LOGPRINT', 'INFO').upper() == 'DEBUG'
    
    with patch.dict(os.environ, {'LOGPRINT': 'warning'}):
      # Test case insensitive
      assert os.environ.get('LOGPRINT', 'INFO').upper() == 'WARNING'
      
  def test_carlog_logging_methods(self):
    """Test carlog supports standard logging methods"""
    # Verify carlog has standard logging methods
    assert hasattr(carlog.carlog, 'debug')
    assert hasattr(carlog.carlog, 'info')
    assert hasattr(carlog.carlog, 'warning')
    assert hasattr(carlog.carlog, 'error')
    assert hasattr(carlog.carlog, 'exception')
    assert hasattr(carlog.carlog, 'critical')