import time
from unittest.mock import Mock, patch
from opendbc.car.ecu_addrs import (
  _is_tester_present_response, get_all_ecu_addrs, get_ecu_addrs
)
from opendbc.car import uds


class TestEcuAddrs:
  def test_is_tester_present_response_success(self):
    """Test identifying successful tester present response"""
    # Test without subaddress
    msg = Mock()
    msg.dat = bytes([0x02, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    assert _is_tester_present_response(msg) is True
    
    # Test with subaddress
    msg.dat = bytes([0x10, 0x02, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00])
    assert _is_tester_present_response(msg, subaddr=0x10) is True

  def test_is_tester_present_response_error(self):
    """Test identifying error tester present response"""
    # Test error response without subaddress
    msg = Mock()
    msg.dat = bytes([0x03, 0x7F, uds.SERVICE_TYPE.TESTER_PRESENT, 0x11, 0x00, 0x00, 0x00, 0x00])
    assert _is_tester_present_response(msg) is True
    
    # Test error response with subaddress
    msg.dat = bytes([0x20, 0x03, 0x7F, uds.SERVICE_TYPE.TESTER_PRESENT, 0x11, 0x00, 0x00, 0x00])
    assert _is_tester_present_response(msg, subaddr=0x20) is True

  def test_is_tester_present_response_invalid(self):
    """Test non-tester present responses return False"""
    msg = Mock()
    
    # Wrong service type
    msg.dat = bytes([0x02, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    assert _is_tester_present_response(msg) is False
    
    # Wrong length
    msg.dat = bytes([0x02, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00])
    assert _is_tester_present_response(msg) is False
    
    # Invalid frame length indicator
    msg.dat = bytes([0x00, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    assert _is_tester_present_response(msg) is False
    
    # Frame length > 7 (multi-frame)
    msg.dat = bytes([0x10, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    assert _is_tester_present_response(msg, subaddr=None) is False

  def test_get_all_ecu_addrs(self):
    """Test get_all_ecu_addrs generates correct query list"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    with patch('opendbc.car.ecu_addrs.get_ecu_addrs') as mock_get_ecu:
      mock_get_ecu.return_value = {(0x7e0, None, 0), (0x7e8, None, 0)}
      
      result = get_all_ecu_addrs(mock_can_recv, mock_can_send, bus=0, timeout=2.0)
      
      # Should call get_ecu_addrs with correct parameters
      mock_get_ecu.assert_called_once()
      args = mock_get_ecu.call_args[0]
      
      # Check queries include both standard and extended addresses
      queries = args[2]
      assert len(queries) == 512  # 256 standard + 256 extended
      assert (0x700, None, 0) in queries
      assert (0x7ff, None, 0) in queries
      assert (0x18da00f1, None, 0) in queries
      
      # Check result
      assert result == {(0x7e0, None, 0), (0x7e8, None, 0)}

  def test_get_ecu_addrs_success(self):
    """Test successful ECU address discovery"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    # Mock CAN packets with tester present responses
    mock_packet1 = Mock()
    mock_packet1.address = 0x7e8
    mock_packet1.src = 0
    mock_packet1.dat = bytes([0x02, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    mock_packet2 = Mock()
    mock_packet2.address = 0x7e0
    mock_packet2.src = 0
    mock_packet2.dat = bytes([0x02, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    # First call clears buffer, subsequent calls return packets
    mock_can_recv.side_effect = [
      None,  # Initial clear
      [[mock_packet1]],  # First response
      [[mock_packet2]],  # Second response
      []  # No more packets
    ]
    
    with patch('time.monotonic', side_effect=[0, 0.1, 0.2, 1.1]):  # Simulate timeout
      with patch('opendbc.car.ecu_addrs.carlog'):
        queries = {(0x7e0, None, 0), (0x7e8, None, 0)}
        responses = queries
        
        result = get_ecu_addrs(mock_can_recv, mock_can_send, queries, responses, timeout=1.0)
        
        # Should send tester present messages
        mock_can_send.assert_called_once()
        sent_msgs = mock_can_send.call_args[0][0]
        assert len(sent_msgs) == 2
        
        # Should return discovered ECUs
        assert result == {(0x7e8, None, 0), (0x7e0, None, 0)}

  def test_get_ecu_addrs_with_subaddress(self):
    """Test ECU discovery with subaddress"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    # Mock response with subaddress
    mock_packet = Mock()
    mock_packet.address = 0x7e0
    mock_packet.src = 0
    mock_packet.dat = bytes([0x10, 0x02, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    mock_can_recv.side_effect = [None, [[mock_packet]], []]
    
    with patch('time.monotonic', side_effect=[0, 0.1, 1.1]):
      with patch('opendbc.car.ecu_addrs.carlog'):
        queries = {(0x7e0, 0x10, 0)}
        responses = queries
        
        result = get_ecu_addrs(mock_can_recv, mock_can_send, queries, responses, timeout=1.0)
        
        assert result == {(0x7e0, 0x10, 0)}

  def test_get_ecu_addrs_empty_frame_warning(self):
    """Test warning on empty remote frame"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    # Mock empty frame
    mock_packet = Mock()
    mock_packet.address = 0x7e0
    mock_packet.src = 0
    mock_packet.dat = b''  # Empty data
    
    mock_can_recv.side_effect = [None, [[mock_packet]], []]
    
    with patch('time.monotonic', side_effect=[0, 0.1, 1.1]):
      with patch('opendbc.car.ecu_addrs.carlog') as mock_carlog:
        queries = {(0x7e0, None, 0)}
        responses = queries
        
        result = get_ecu_addrs(mock_can_recv, mock_can_send, queries, responses, timeout=1.0)
        
        # Should log warning about empty frame
        mock_carlog.warning.assert_called_with("ECU addr scan: skipping empty remote frame")
        # Should not discover any ECUs
        assert result == set()

  def test_get_ecu_addrs_duplicate_response(self):
    """Test handling of duplicate ECU responses"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    # Mock duplicate responses from same ECU
    mock_packet = Mock()
    mock_packet.address = 0x7e0
    mock_packet.src = 0
    mock_packet.dat = bytes([0x02, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    mock_can_recv.side_effect = [None, [[mock_packet]], [[mock_packet]], []]  # Same packet twice
    
    with patch('time.monotonic', side_effect=[0, 0.1, 0.2, 1.1]):
      with patch('opendbc.car.ecu_addrs.carlog') as mock_carlog:
        queries = {(0x7e0, None, 0)}
        responses = queries
        
        result = get_ecu_addrs(mock_can_recv, mock_can_send, queries, responses, timeout=1.0)
        
        # Should log duplicate
        mock_carlog.debug.assert_any_call("Duplicate ECU address: 0x7e0")
        # Should still only have one ECU in result
        assert result == {(0x7e0, None, 0)}

  def test_get_ecu_addrs_exception_handling(self):
    """Test exception handling during ECU scan"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    # Mock exception during receive
    mock_can_recv.side_effect = Exception("CAN error")
    
    with patch('opendbc.car.ecu_addrs.carlog') as mock_carlog:
      queries = {(0x7e0, None, 0)}
      responses = queries
      
      result = get_ecu_addrs(mock_can_recv, mock_can_send, queries, responses, timeout=1.0)
      
      # Should log exception
      mock_carlog.exception.assert_called_with("ECU addr scan exception")
      # Should return empty set
      assert result == set()

  def test_get_ecu_addrs_non_matching_response(self):
    """Test ignoring responses not in expected set"""
    mock_can_recv = Mock()
    mock_can_send = Mock()
    
    # Mock response from unexpected address
    mock_packet = Mock()
    mock_packet.address = 0x7e8  # Not in queries
    mock_packet.src = 0
    mock_packet.dat = bytes([0x02, uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    mock_can_recv.side_effect = [None, [[mock_packet]], []]
    
    with patch('time.monotonic', side_effect=[0, 0.1, 1.1]):
      with patch('opendbc.car.ecu_addrs.carlog'):
        queries = {(0x7e0, None, 0)}  # Only expecting 0x7e0
        responses = queries
        
        result = get_ecu_addrs(mock_can_recv, mock_can_send, queries, responses, timeout=1.0)
        
        # Should not include unexpected response
        assert result == set()