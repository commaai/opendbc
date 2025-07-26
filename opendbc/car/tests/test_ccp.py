from opendbc.car.ccp import (
  COMMAND_CODE, BYTE_ORDER, CommandTimeoutError, 
  CommandCounterError, CommandResponseError, ExchangeStationIdsReturn,
  COMMAND_RETURN_CODES
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