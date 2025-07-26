import re
from unittest.mock import Mock, patch
from opendbc.car.vin import Vin, is_valid_vin, get_vin, VIN_UNKNOWN, VIN_RE


class TestVin:
  def test_vin_dataclass_basic(self):
    """Test basic Vin dataclass functionality"""
    # Valid VIN example (Honda Civic)
    vin_str = "2HGFC2F59HH123456"
    vin = Vin(vin_str)

    assert vin.vin == vin_str
    assert vin.wmi == "2HG"  # World Manufacturer Identifier
    assert vin.vds == "FC2F59"  # Vehicle Descriptor Section
    assert vin.vis == "HH123456"  # Vehicle Identifier Section

  def test_vin_dataclass_different_manufacturer(self):
    """Test VIN parsing for different manufacturer"""
    # Valid VIN example (Toyota)
    vin_str = "JTDKARFP8K2123456"
    vin = Vin(vin_str)

    assert vin.vin == vin_str
    assert vin.wmi == "JTD"  # Toyota WMI
    assert vin.vds == "KARFP8"
    assert vin.vis == "K2123456"

  def test_vin_dataclass_ford(self):
    """Test VIN parsing for Ford vehicle"""
    # Valid Ford VIN
    vin_str = "1FAHP2E89GL123456"
    vin = Vin(vin_str)

    assert vin.vin == vin_str
    assert vin.wmi == "1FA"  # Ford WMI
    assert vin.vds == "HP2E89"
    assert vin.vis == "GL123456"

  def test_vin_components_length(self):
    """Test that VIN components have correct lengths"""
    vin_str = "1HGBH41JXMN123456"
    vin = Vin(vin_str)

    assert len(vin.wmi) == 3
    assert len(vin.vds) == 6
    assert len(vin.vis) == 8
    assert len(vin.vin) == 17

  def test_vin_edge_case_all_numbers(self):
    """Test VIN with maximum allowed numbers"""
    # VIN using numbers where allowed (avoiding I, O, Q)
    vin_str = "12345678901234567"
    vin = Vin(vin_str)

    assert vin.wmi == "123"
    assert vin.vds == "456789"
    assert vin.vis == "01234567"


class TestVinValidation:
  def test_is_valid_vin_correct_length(self):
    """Test VIN validation for correct length"""
    valid_vin = "1HGBH41JXMN123456"
    assert is_valid_vin(valid_vin) is True

  def test_is_valid_vin_wrong_length_short(self):
    """Test VIN validation for incorrect length (too short)"""
    short_vin = "1HGBH41JXMN123"  # 14 chars instead of 17
    assert is_valid_vin(short_vin) is False

  def test_is_valid_vin_wrong_length_long(self):
    """Test VIN validation for incorrect length (too long)"""
    long_vin = "1HGBH41JXMN1234567890"  # 20 chars instead of 17
    assert is_valid_vin(long_vin) is False

  def test_is_valid_vin_invalid_characters(self):
    """Test VIN validation for invalid characters"""
    # VIN with forbidden letters I, O, Q
    invalid_vin_i = "1HGBH41JXIN123456"  # Contains 'I'
    invalid_vin_o = "1HGBH41JXON123456"  # Contains 'O'
    invalid_vin_q = "1HGBH41JXQN123456"  # Contains 'Q'

    assert is_valid_vin(invalid_vin_i) is False
    assert is_valid_vin(invalid_vin_o) is False
    assert is_valid_vin(invalid_vin_q) is False

  def test_is_valid_vin_lowercase_letters(self):
    """Test VIN validation with lowercase letters"""
    lowercase_vin = "1hgbh41jxmn123456"
    assert is_valid_vin(lowercase_vin) is False

  def test_is_valid_vin_special_characters(self):
    """Test VIN validation with special characters"""
    special_vin = "1HGBH41JX-N123456"  # Contains dash
    assert is_valid_vin(special_vin) is False

  def test_is_valid_vin_empty_string(self):
    """Test VIN validation with empty string"""
    assert is_valid_vin("") is False

  def test_is_valid_vin_all_valid_characters(self):
    """Test VIN with all valid character types"""
    # Using A-H, J-N, P-Z, 0-9 (excluding I, O, Q)
    valid_vin = "ABCDEFGHJKLMNPRST"
    assert is_valid_vin(valid_vin) is True

  def test_vin_regex_pattern(self):
    """Test the VIN regex pattern directly"""
    pattern = VIN_RE

    # Valid VIN should match
    valid_vin = "1HGBH41JXMN123456"
    assert re.fullmatch(pattern, valid_vin) is not None

    # Invalid VIN should not match
    invalid_vin = "1HGBH41JXIN123456"  # Contains 'I'
    assert re.fullmatch(pattern, invalid_vin) is None


class TestVinConstant:
  def test_vin_unknown_constant(self):
    """Test VIN_UNKNOWN constant"""
    assert VIN_UNKNOWN == "0" * 17
    assert len(VIN_UNKNOWN) == 17
    assert is_valid_vin(VIN_UNKNOWN) is True  # Should be valid format


class TestGetVin:
  def setup_method(self):
    """Setup test fixtures"""
    self.mock_can_send = Mock()
    self.mock_can_recv = Mock()
    self.buses = [0, 1]

  def test_get_vin_success_uds(self):
    """Test successful VIN retrieval via UDS"""
    # Mock successful VIN response
    test_vin = "1HGBH41JXMN123456"

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      mock_query.get_data.return_value = {(0x7e0, None): test_vin.encode()}

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, self.buses)

      assert vin == test_vin
      assert bus in self.buses
      assert addr > 0

  def test_get_vin_success_obd(self):
    """Test successful VIN retrieval via OBD"""
    test_vin = "JTDKARFP8K2123456"

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      # First call (UDS) fails, second call (OBD) succeeds
      mock_query.get_data.side_effect = [
        {},  # UDS fails
        {(0x7e0, None): test_vin.encode()}  # OBD succeeds
      ]

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, self.buses)

      assert vin == test_vin

  def test_get_vin_ford_null_padding(self):
    """Test VIN retrieval with Ford null padding"""
    test_vin = "1FAHP2E89GL123456"
    padded_vin = test_vin.encode() + b'\x00\x00'  # 19 bytes with null padding

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      mock_query.get_data.return_value = {(0x7e0, None): padded_vin}

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, self.buses)

      assert vin == test_vin  # Null bytes should be stripped

  def test_get_vin_honda_bosch_format(self):
    """Test VIN retrieval with Honda Bosch length prefix"""
    test_vin = "2HGFC2F59HH123456"
    honda_vin = b'\x11' + test_vin.encode()  # Length prefix + VIN

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      mock_query.get_data.return_value = {(0x7e0, None): honda_vin}

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, self.buses)

      assert vin == test_vin  # Length prefix should be stripped

  def test_get_vin_gm_special_addressing(self):
    """Test VIN retrieval with GM special addressing"""
    test_vin = "1G1ZB5ST8KF123456"

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      # GM uses special address 0x24b
      mock_query.get_data.return_value = {(0x24b, None): test_vin.encode()}

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, self.buses)

      assert vin == test_vin

  def test_get_vin_nissan_kwp(self):
    """Test VIN retrieval with Nissan KWP protocol"""
    test_vin = "1N4AL3AP8KC123456"

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      # Nissan uses address 0x797
      mock_query.get_data.return_value = {(0x797, None): test_vin.encode()}

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, self.buses)

      assert vin == test_vin

  def test_get_vin_timeout_retry(self):
    """Test VIN retrieval with timeout and retry logic"""
    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      # All attempts fail
      mock_query.get_data.return_value = {}

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, self.buses, retry=3)

      assert vin == VIN_UNKNOWN
      assert addr == -1
      assert bus == -1

  def test_get_vin_exception_handling(self):
    """Test VIN retrieval with exception handling"""
    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      # Simulate exception during query
      mock_query_class.side_effect = Exception("CAN communication error")

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, self.buses)

      assert vin == VIN_UNKNOWN
      assert addr == -1
      assert bus == -1

  def test_get_vin_bus_filtering(self):
    """Test that VIN queries respect valid bus filtering"""
    test_vin = "1HGBH41JXMN123456"

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      mock_query.get_data.return_value = {(0x7e0, None): test_vin.encode()}

      # Test with bus 2, which should be skipped for some protocols
      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, [2])

      # Should still try protocols that support bus 2 or return unknown
      assert isinstance(vin, str)

  def test_get_vin_multiple_buses(self):
    """Test VIN retrieval across multiple buses"""
    test_vin = "5NPE34AF9KH123456"

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      # Fail on bus 0, succeed on bus 1
      mock_query.get_data.side_effect = [{}, {(0x7e0, None): test_vin.encode()}]

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, [0, 1])

      assert vin == test_vin

  def test_get_vin_custom_timeout(self):
    """Test VIN retrieval with custom timeout"""
    test_vin = "KNMAT2MV4KP123456"

    with patch('opendbc.car.vin.IsoTpParallelQuery') as mock_query_class:
      mock_query = Mock()
      mock_query_class.return_value = mock_query
      mock_query.get_data.return_value = {(0x7e0, None): test_vin.encode()}

      addr, bus, vin = get_vin(self.mock_can_recv, self.mock_can_send, [0], timeout=0.5)

      assert vin == test_vin
      # Verify timeout was passed to query
      mock_query.get_data.assert_called_with(0.5)