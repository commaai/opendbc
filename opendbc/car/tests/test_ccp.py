from unittest.mock import Mock, patch
from opendbc.car.ccp import (
  CcpClient, COMMAND_CODE, BYTE_ORDER, CommandTimeoutError,
  CommandCounterError, CommandResponseError, ExchangeStationIdsReturn,
  GetDaqListSizeReturn, GetSessionStatusReturn, DiagnosticServiceReturn,
  ActionServiceReturn, COMMAND_RETURN_CODES
)


class TestCcp:
  def test_command_codes_exist(self):
    """Test that command codes are properly defined"""
    assert COMMAND_CODE.CONNECT == 0x01
    assert COMMAND_CODE.DISCONNECT == 0x07
    assert COMMAND_CODE.UPLOAD == 0x04
    assert COMMAND_CODE.DNLOAD == 0x03

  def test_byte_order_enum(self):
    """Test byte order enumeration"""
    assert BYTE_ORDER.LITTLE_ENDIAN.value == '<'
    assert BYTE_ORDER.BIG_ENDIAN.value == '>'

  def test_command_return_codes(self):
    """Test command return codes dictionary"""
    assert 0x00 in COMMAND_RETURN_CODES
    assert COMMAND_RETURN_CODES[0x00] == "acknowledge / no error"
    assert 0x30 in COMMAND_RETURN_CODES
    assert COMMAND_RETURN_CODES[0x30] == "unknown command"

  def test_exception_classes(self):
    """Test exception classes can be instantiated"""
    timeout_err = CommandTimeoutError("test timeout")
    assert str(timeout_err) == "test timeout"

    counter_err = CommandCounterError("test counter")
    assert str(counter_err) == "test counter"

    response_err = CommandResponseError("test response", 0x31)
    assert str(response_err) == "test response"
    assert response_err.return_code == 0x31

  def test_dataclass_creation(self):
    """Test dataclass can be created"""
    result = ExchangeStationIdsReturn(id_length=8, data_type=1, available=1, protected=0)
    assert result.id_length == 8
    assert result.data_type == 1


class TestCcpClient:
  def create_mock_panda(self):
    """Create a mock panda device"""
    mock = Mock()
    mock.can_clear = Mock()
    mock.can_send = Mock()
    mock.can_recv = Mock()
    return mock

  def test_ccp_client_init(self):
    """Test CcpClient initialization"""
    panda = self.create_mock_panda()
    client = CcpClient(panda, tx_addr=0x700, rx_addr=0x701, bus=1,
                      byte_order=BYTE_ORDER.LITTLE_ENDIAN, debug=True)

    assert client.tx_addr == 0x700
    assert client.rx_addr == 0x701
    assert client.can_bus == 1
    assert client.byte_order == BYTE_ORDER.LITTLE_ENDIAN
    assert client.debug is True
    assert client._command_counter == -1

  def test_send_cro_increments_counter(self):
    """Test _send_cro increments command counter"""
    panda = self.create_mock_panda()
    client = CcpClient(panda, 0x700, 0x701)

    # Initial counter is -1
    assert client._command_counter == -1

    client._send_cro(COMMAND_CODE.CONNECT, b'\x12\x34')
    assert client._command_counter == 0

    client._send_cro(COMMAND_CODE.DISCONNECT)
    assert client._command_counter == 1

  def test_recv_dto_timeout(self):
    """Test _recv_dto timeout handling"""
    panda = self.create_mock_panda()
    panda.can_recv.return_value = []  # No messages

    client = CcpClient(panda, 0x700, 0x701)

    with patch('time.time', side_effect=[0, 0.5, 1.1]):
      try:
        client._recv_dto(1.0)
        assert False, "Should have raised timeout"
      except CommandTimeoutError as e:
        assert "timeout waiting for response" in str(e)

  def test_connect_command(self):
    """Test connect command"""
    panda = self.create_mock_panda()
    response = bytes([0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    panda.can_recv.return_value = [(0x701, response, 0)]

    client = CcpClient(panda, 0x700, 0x701)

    # Should not raise exception
    client.connect(0x1234)

    # Check command was sent
    panda.can_send.assert_called_once()
    sent_data = panda.can_send.call_args[0][1]
    assert sent_data[0] == COMMAND_CODE.CONNECT
    assert sent_data[1] == 0  # counter

  def test_disconnect_command(self):
    """Test disconnect command"""
    panda = self.create_mock_panda()
    response = bytes([0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    panda.can_recv.return_value = [(0x701, response, 0)]

    client = CcpClient(panda, 0x700, 0x701)

    # Test temporary disconnect
    client.disconnect(0x1234, temporary=True)

    # Check command format
    sent_data = panda.can_send.call_args[0][1]
    assert sent_data[0] == COMMAND_CODE.DISCONNECT
    assert sent_data[2] == 0  # temporary flag (inverted)

  def test_invalid_parameters(self):
    """Test parameter validation"""
    panda = self.create_mock_panda()
    client = CcpClient(panda, 0x700, 0x701)

    # Test station address > 65535
    try:
      client.connect(70000)
      assert False, "Should raise ValueError"
    except ValueError as e:
      assert "station address must be less than 65536" in str(e)

    # Test resource mask > 255
    try:
      client.get_seed(300)
      assert False, "Should raise ValueError"
    except ValueError as e:
      assert "resource mask must be less than 256" in str(e)

  def test_command_response_error(self):
    """Test command error response handling"""
    panda = self.create_mock_panda()
    # Error response: unknown command (0x30)
    response = bytes([0xFF, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    panda.can_recv.return_value = [(0x701, response, 0)]

    client = CcpClient(panda, 0x700, 0x701)

    try:
      client.connect(0x1234)
      assert False, "Should raise CommandResponseError"
    except CommandResponseError as e:
      assert e.return_code == 0x30
      assert "unknown command" in str(e)

  def test_dataclass_returns(self):
    """Test methods that return dataclasses"""
    panda = self.create_mock_panda()
    client = CcpClient(panda, 0x700, 0x701)

    # Test GetDaqListSizeReturn
    daq_result = GetDaqListSizeReturn(list_size=10, first_pid=5)
    assert daq_result.list_size == 10
    assert daq_result.first_pid == 5

    # Test GetSessionStatusReturn
    session_result = GetSessionStatusReturn(status=1, info=0xAB)
    assert session_result.status == 1
    assert session_result.info == 0xAB

    # Test DiagnosticServiceReturn
    diag_result = DiagnosticServiceReturn(length=4, type=2)
    assert diag_result.length == 4
    assert diag_result.type == 2

    # Test ActionServiceReturn
    action_result = ActionServiceReturn(length=6, type=3)
    assert action_result.length == 6
    assert action_result.type == 3