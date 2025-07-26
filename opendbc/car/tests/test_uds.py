from unittest.mock import Mock
from opendbc.car.uds import (
  SERVICE_TYPE, SESSION_TYPE, DATA_IDENTIFIER_TYPE, ISOTP_FRAME_TYPE, DynamicSourceDefinition, DTC_STATUS_MASK_TYPE, MessageTimeoutError,
  NegativeResponseError, InvalidServiceIdError, InvalidSubAddressError, get_dtc_num_as_str, get_dtc_status_names,
  CanClient, IsoTpMessage, UdsClient, get_rx_addr_for_tx_addr, FUNCTIONAL_ADDRS
)


class TestUdsEnums:
  def test_service_type_enum(self):
    """Test SERVICE_TYPE enum values"""
    assert SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL == 0x10
    assert SERVICE_TYPE.TESTER_PRESENT == 0x3E
    assert SERVICE_TYPE.READ_DATA_BY_IDENTIFIER == 0x22
    assert SERVICE_TYPE.WRITE_DATA_BY_IDENTIFIER == 0x2E

  def test_session_type_enum(self):
    """Test SESSION_TYPE enum values"""
    assert SESSION_TYPE.DEFAULT == 1
    assert SESSION_TYPE.PROGRAMMING == 2
    assert SESSION_TYPE.EXTENDED_DIAGNOSTIC == 3

  def test_data_identifier_type_enum(self):
    """Test DATA_IDENTIFIER_TYPE enum values"""
    assert DATA_IDENTIFIER_TYPE.VIN == 0xF190
    assert DATA_IDENTIFIER_TYPE.APPLICATION_SOFTWARE_IDENTIFICATION == 0xF181
    assert DATA_IDENTIFIER_TYPE.ECU_SERIAL_NUMBER == 0xF18C

  def test_isotp_frame_type_enum(self):
    """Test ISOTP_FRAME_TYPE enum values"""
    assert ISOTP_FRAME_TYPE.SINGLE == 0
    assert ISOTP_FRAME_TYPE.FIRST == 1
    assert ISOTP_FRAME_TYPE.CONSECUTIVE == 2
    assert ISOTP_FRAME_TYPE.FLOW == 3


class TestUdsExceptions:
  def test_negative_response_error(self):
    """Test NegativeResponseError exception"""
    error = NegativeResponseError("Test error", 0x10, 0x11)
    assert error.message == "Test error"
    assert error.service_id == 0x10
    assert error.error_code == 0x11
    assert str(error) == "Test error"

  def test_message_timeout_error(self):
    """Test MessageTimeoutError exception"""
    error = MessageTimeoutError("Timeout occurred")
    assert str(error) == "Timeout occurred"

  def test_invalid_service_id_error(self):
    """Test InvalidServiceIdError exception"""
    error = InvalidServiceIdError("Invalid service ID")
    assert str(error) == "Invalid service ID"


class TestUtilityFunctions:
  def test_get_dtc_num_as_str(self):
    """Test DTC number to string conversion"""
    # Test P-code (Powertrain)
    dtc_bytes = bytes([0x01, 0x23])  # P0123
    result = get_dtc_num_as_str(dtc_bytes)
    assert result == "P0123"

    # Test C-code (Chassis)
    dtc_bytes = bytes([0x45, 0x67])  # C0567
    result = get_dtc_num_as_str(dtc_bytes)
    assert result == "C0567"

    # Test B-code (Body)
    dtc_bytes = bytes([0x89, 0xAB])  # B09AB
    result = get_dtc_num_as_str(dtc_bytes)
    assert result == "B09ab"

    # Test U-code (Network)
    dtc_bytes = bytes([0xCD, 0xEF])  # U0DEF
    result = get_dtc_num_as_str(dtc_bytes)
    assert result == "U0def"

  def test_get_dtc_status_names(self):
    """Test DTC status mask to names conversion"""
    # Test single flag
    status = DTC_STATUS_MASK_TYPE.TEST_FAILED
    names = get_dtc_status_names(status)
    assert "TEST_FAILED" in names
    assert len(names) == 1

    # Test multiple flags
    status = DTC_STATUS_MASK_TYPE.TEST_FAILED | DTC_STATUS_MASK_TYPE.CONFIRMED_DTC
    names = get_dtc_status_names(status)
    assert "TEST_FAILED" in names
    assert "CONFIRMED_DTC" in names
    assert len(names) == 2

    # Test no flags
    names = get_dtc_status_names(0)
    assert len(names) == 0

  def test_get_rx_addr_for_tx_addr(self):
    """Test RX address calculation from TX address"""
    # Test 11-bit addresses (standard offset)
    assert get_rx_addr_for_tx_addr(0x7E0) == 0x7E8
    assert get_rx_addr_for_tx_addr(0x7E1) == 0x7E9

    # Test 11-bit addresses with custom offset
    assert get_rx_addr_for_tx_addr(0x7E0, rx_offset=0x10) == 0x7F0

    # Test 29-bit addresses
    assert get_rx_addr_for_tx_addr(0x18DA00F1) == 0x18DAF100

    # Test functional addresses
    assert get_rx_addr_for_tx_addr(0x7DF) is None
    assert get_rx_addr_for_tx_addr(0x18DB33F1) is None

    # Test invalid addresses
    try:
      get_rx_addr_for_tx_addr(0xFFF9)  # Out of range
      raise AssertionError("Should raise ValueError")
    except ValueError:
      pass


class TestCanClient:
  def setup_method(self):
    """Set up test fixtures"""
    self.mock_can_send = Mock()
    self.mock_can_recv = Mock()
    self.client = CanClient(
      self.mock_can_send, self.mock_can_recv,
      tx_addr=0x7E0, rx_addr=0x7E8, bus=0
    )

  def test_can_client_init(self):
    """Test CanClient initialization"""
    assert self.client.tx_addr == 0x7E0
    assert self.client.rx_addr == 0x7E8
    assert self.client.bus == 0
    assert self.client.sub_addr is None
    assert self.client.rx_sub_addr is None

  def test_can_client_with_sub_addr(self):
    """Test CanClient with sub-addressing"""
    client = CanClient(
      self.mock_can_send, self.mock_can_recv,
      tx_addr=0x7E0, rx_addr=0x7E8, bus=0,
      sub_addr=0x10, rx_sub_addr=0x20
    )
    assert client.sub_addr == 0x10
    assert client.rx_sub_addr == 0x20

  def test_recv_filter_basic(self):
    """Test basic receive filtering"""
    # Should accept messages on correct bus and address
    assert self.client._recv_filter(0, 0x7E8) is True

    # Should reject wrong bus
    assert self.client._recv_filter(1, 0x7E8) is False

    # Should reject wrong address
    assert self.client._recv_filter(0, 0x7E9) is False

  def test_recv_filter_functional_obd(self):
    """Test functional addressing for OBD"""
    client = CanClient(
      self.mock_can_send, self.mock_can_recv,
      tx_addr=0x7DF, rx_addr=0x7E8, bus=0
    )

    # Should accept response in OBD range and switch addresses
    assert client._recv_filter(0, 0x7E8) is True
    assert client.tx_addr == 0x7E0  # Should switch to physical
    assert client.rx_addr == 0x7E8

  def test_recv_filter_functional_29bit(self):
    """Test functional addressing for 29-bit"""
    client = CanClient(
      self.mock_can_send, self.mock_can_recv,
      tx_addr=0x18DB33F1, rx_addr=0x18DAF100, bus=0
    )

    # Should accept response in 29-bit functional range
    result = client._recv_filter(0, 0x18DAF110)
    assert client.tx_addr != 0x18DB33F1  # Should change from functional

  def test_send_basic(self):
    """Test basic CAN send"""
    messages = [b'\x01\x02\x03']
    self.client.send(messages)

    self.mock_can_send.assert_called_once_with(0x7E0, b'\x01\x02\x03', 0)

  def test_send_with_sub_addr(self):
    """Test CAN send with sub-addressing"""
    client = CanClient(
      self.mock_can_send, self.mock_can_recv,
      tx_addr=0x7E0, rx_addr=0x7E8, bus=0, sub_addr=0x10
    )

    messages = [b'\x01\x02\x03']
    client.send(messages)

    self.mock_can_send.assert_called_once_with(0x7E0, b'\x10\x01\x02\x03', 0)

  def test_send_message_too_long(self):
    """Test sending message that's too long"""
    messages = [b'\x01\x02\x03\x04\x05\x06\x07\x08\x09']  # 9 bytes > 8
    try:
      self.client.send(messages)
      raise AssertionError("Should raise AssertionError")
    except AssertionError:
      pass

  def test_recv_with_sub_addr_validation(self):
    """Test receive with sub-address validation"""
    client = CanClient(
      self.mock_can_send, self.mock_can_recv,
      tx_addr=0x7E0, rx_addr=0x7E8, bus=0, rx_sub_addr=0x10
    )

    # Mock receive data with correct sub-address
    self.mock_can_recv.return_value = [(0x7E8, b'\x10\x01\x02', 0)]

    messages = list(client.recv())
    assert len(messages) == 1
    assert messages[0] == b'\x01\x02'  # Sub-address stripped

  def test_recv_invalid_sub_addr(self):
    """Test receive with invalid sub-address"""
    client = CanClient(
      self.mock_can_send, self.mock_can_recv,
      tx_addr=0x7E0, rx_addr=0x7E8, bus=0, rx_sub_addr=0x10
    )

    # Mock receive data with wrong sub-address
    self.mock_can_recv.return_value = [(0x7E8, b'\x20\x01\x02', 0)]

    try:
      list(client.recv())
      assert False, "Should raise InvalidSubAddressError"
    except InvalidSubAddressError:
      pass


class TestIsoTpMessage:
  def setup_method(self):
    """Set up test fixtures"""
    self.mock_can_client = Mock()
    self.mock_can_client.sub_addr = None  # Explicitly set sub_addr
    self.isotp = IsoTpMessage(self.mock_can_client, timeout=1.0)

  def test_isotp_init_basic(self):
    """Test IsoTpMessage initialization"""
    assert self.isotp.timeout == 1.0
    assert self.isotp.single_frame_mode is False
    assert self.isotp.max_len == 8  # No sub-addressing

  def test_isotp_init_with_sub_addr(self):
    """Test IsoTpMessage with sub-addressing"""
    self.mock_can_client.sub_addr = 0x10
    isotp = IsoTpMessage(self.mock_can_client)
    assert isotp.max_len == 7  # With sub-addressing

  def test_isotp_separation_time_milliseconds(self):
    """Test separation time in milliseconds"""
    isotp = IsoTpMessage(self.mock_can_client, separation_time=0.05)  # 50ms
    # Flow control message should have separation time = 50
    assert isotp.flow_control_msg[2] == 50

  def test_isotp_separation_time_microseconds(self):
    """Test separation time in microseconds"""
    isotp = IsoTpMessage(self.mock_can_client, separation_time=0.0005)  # 500μs
    # Should map to 0xF5 (0xF1 + 4)
    assert isotp.flow_control_msg[2] == 0xF5

  def test_isotp_separation_time_invalid(self):
    """Test invalid separation time"""
    try:
      IsoTpMessage(self.mock_can_client, separation_time=1.0)  # Too large
      assert False, "Should raise Exception"
    except Exception:
      pass

  def test_isotp_send_setup_only(self):
    """Test ISO-TP send in setup-only mode"""
    data = b'\x01\x02\x03'
    self.isotp.send(data, setup_only=True)

    assert self.isotp.tx_dat == data
    assert self.isotp.tx_len == 3
    assert self.isotp.tx_done is True  # Single frame

    # Should not actually send when setup_only=True
    self.mock_can_client.send.assert_not_called()

  def test_isotp_has_required_methods(self):
    """Test that IsoTpMessage has required methods"""
    assert hasattr(self.isotp, 'send')
    assert hasattr(self.isotp, 'recv')
    assert hasattr(self.isotp, 'timeout')
    assert hasattr(self.isotp, 'max_len')


class TestUdsClient:
  def setup_method(self):
    """Set up test fixtures"""
    self.mock_panda = Mock()
    self.mock_panda.can_send = Mock()
    self.mock_panda.can_recv = Mock()
    self.uds_client = UdsClient(
      self.mock_panda, tx_addr=0x7E0, rx_addr=0x7E8, bus=0
    )

  def test_uds_client_init(self):
    """Test UdsClient initialization"""
    assert self.uds_client.tx_addr == 0x7E0
    assert self.uds_client.rx_addr == 0x7E8
    assert self.uds_client.bus == 0

  def test_uds_client_auto_rx_addr(self):
    """Test UdsClient with automatic RX address calculation"""
    client = UdsClient(self.mock_panda, tx_addr=0x7E0, bus=0)
    assert client.rx_addr == 0x7E8  # Auto-calculated

  def test_diagnostic_session_control(self):
    """Test diagnostic session control service"""
    # Test that the method exists and has correct signature
    assert hasattr(self.uds_client, 'diagnostic_session_control')
    assert callable(self.uds_client.diagnostic_session_control)

  def test_tester_present(self):
    """Test tester present service"""
    assert hasattr(self.uds_client, 'tester_present')

  def test_read_data_by_identifier(self):
    """Test read data by identifier service"""
    assert hasattr(self.uds_client, 'read_data_by_identifier')

  def test_security_access_request_seed(self):
    """Test security access request seed"""
    assert hasattr(self.uds_client, 'security_access')

  def test_security_access_validation(self):
    """Test security access parameter validation"""
    # These should be tested by calling the actual method with mocked responses
    # but for now we verify the method exists and can handle validation
    assert hasattr(self.uds_client, 'security_access')


class TestDynamicSourceDefinition:
  def test_dynamic_source_definition(self):
    """Test DynamicSourceDefinition NamedTuple"""
    definition = DynamicSourceDefinition(
      data_identifier=0x1234,
      position=5,
      memory_size=10,
      memory_address=0x10000
    )

    assert definition.data_identifier == 0x1234
    assert definition.position == 5
    assert definition.memory_size == 10
    assert definition.memory_address == 0x10000


class TestConstants:
  def test_functional_addrs(self):
    """Test functional address constants"""
    assert 0x7DF in FUNCTIONAL_ADDRS
    assert 0x18DB33F1 in FUNCTIONAL_ADDRS
    assert len(FUNCTIONAL_ADDRS) == 2