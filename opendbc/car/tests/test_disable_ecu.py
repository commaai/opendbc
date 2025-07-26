from unittest.mock import Mock, patch, MagicMock
from opendbc.car.disable_ecu import disable_ecu, EXT_DIAG_REQUEST, EXT_DIAG_RESPONSE, COM_CONT_RESPONSE


class TestDisableEcu:
  def test_disable_ecu_success_first_try(self):
    """Test successful ECU disable on first attempt"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    # Mock IsoTpParallelQuery
    with patch('opendbc.car.disable_ecu.IsoTpParallelQuery') as mock_query_class:
      # Create mock query instances
      mock_query1 = Mock()
      mock_query2 = Mock()
      
      # First query returns success for extended diagnostic
      mock_query1.get_data.return_value = {(0x7d0, None): b'some_response'}
      # Second query for communication control
      mock_query2.get_data.return_value = {}
      
      # Configure the class to return our mocks in sequence
      mock_query_class.side_effect = [mock_query1, mock_query2]
      
      with patch('opendbc.car.disable_ecu.carlog') as mock_carlog:
        result = disable_ecu(mock_can_recv, mock_can_send)
        
        # Should succeed
        assert result is True
        
        # Should log appropriate messages
        mock_carlog.warning.assert_any_call("ecu disable ('0x7d0', None) ...")
        mock_carlog.warning.assert_any_call("communication control disable tx/rx ...")
        mock_carlog.warning.assert_any_call("ecu disabled")
        
        # Should not log errors
        mock_carlog.error.assert_not_called()

  def test_disable_ecu_custom_parameters(self):
    """Test ECU disable with custom bus, address, and sub_address"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    with patch('opendbc.car.disable_ecu.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query.get_data.return_value = {(0x7e0, 0x01): b'response'}
      mock_query_class.return_value = mock_query
      
      with patch('opendbc.car.disable_ecu.carlog'):
        result = disable_ecu(
          mock_can_recv, mock_can_send, 
          bus=1, addr=0x7e0, sub_addr=0x01,
          com_cont_req=b'\x28\x83\x02', timeout=0.5
        )
        
        # Should create query with correct parameters
        mock_query_class.assert_any_call(
          mock_can_send, mock_can_recv, 1, [(0x7e0, 0x01)], 
          [EXT_DIAG_REQUEST], [EXT_DIAG_RESPONSE]
        )

  def test_disable_ecu_exception_retry(self):
    """Test ECU disable retries on exception"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    with patch('opendbc.car.disable_ecu.IsoTpParallelQuery') as mock_query_class:
      # First attempt raises exception, second succeeds
      mock_query_fail = Mock()
      mock_query_fail.get_data.side_effect = Exception("Connection error")
      
      mock_query_success = Mock()
      mock_query_success.get_data.return_value = {(0x7d0, None): b'response'}
      
      mock_query_class.side_effect = [
        mock_query_fail,  # First attempt fails
        mock_query_success,  # Second attempt diagnostic
        mock_query_success   # Second attempt comm control
      ]
      
      with patch('opendbc.car.disable_ecu.carlog') as mock_carlog:
        result = disable_ecu(mock_can_recv, mock_can_send, retry=2)
        
        # Should succeed on retry
        assert result is True
        
        # Should log exception and retry
        mock_carlog.exception.assert_called_once_with("ecu disable exception")
        mock_carlog.error.assert_called_once_with("ecu disable retry (1) ...")

  def test_disable_ecu_all_retries_fail(self):
    """Test ECU disable fails after all retries exhausted"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    with patch('opendbc.car.disable_ecu.IsoTpParallelQuery') as mock_query_class:
      # All attempts fail
      mock_query_class.side_effect = Exception("Persistent error")
      
      with patch('opendbc.car.disable_ecu.carlog') as mock_carlog:
        result = disable_ecu(mock_can_recv, mock_can_send, retry=3)
        
        # Should fail
        assert result is False
        
        # Should log all retries and final failure
        assert mock_carlog.exception.call_count == 3
        assert mock_carlog.error.call_count == 4  # 3 retries + 1 final failure
        mock_carlog.error.assert_any_call("ecu disable failed")

  def test_disable_ecu_no_response_from_diagnostic(self):
    """Test ECU disable when no response from diagnostic query"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    with patch('opendbc.car.disable_ecu.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      # Empty response from diagnostic query
      mock_query.get_data.return_value = {}
      mock_query_class.return_value = mock_query
      
      with patch('opendbc.car.disable_ecu.carlog') as mock_carlog:
        result = disable_ecu(mock_can_recv, mock_can_send, retry=1)
        
        # Should fail since no ECU responded
        assert result is False
        
        # Should not reach communication control stage
        assert mock_query_class.call_count == 1

  def test_disable_ecu_timeout_parameter(self):
    """Test timeout parameter is properly passed to query"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    custom_timeout = 2.5
    
    with patch('opendbc.car.disable_ecu.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query.get_data.return_value = {(0x7d0, None): b'response'}
      mock_query_class.return_value = mock_query
      
      with patch('opendbc.car.disable_ecu.carlog'):
        disable_ecu(mock_can_recv, mock_can_send, timeout=custom_timeout)
        
        # First query should use custom timeout
        mock_query.get_data.assert_any_call(custom_timeout)
        # Second query (comm control) always uses 0 timeout
        mock_query.get_data.assert_any_call(0)