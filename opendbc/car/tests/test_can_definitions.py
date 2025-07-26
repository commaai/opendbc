from opendbc.car.can_definitions import CanData, CanSendCallable, CanRecvCallable


class TestCanDefinitions:
  def test_can_data_namedtuple(self):
    """Test CanData NamedTuple creation and access"""
    can_msg = CanData(
      address=0x123,
      dat=b'\x01\x02\x03\x04',
      src=0
    )

    assert can_msg.address == 0x123
    assert can_msg.dat == b'\x01\x02\x03\x04'
    assert can_msg.src == 0

    # Test tuple-like access
    assert can_msg[0] == 0x123
    assert can_msg[1] == b'\x01\x02\x03\x04'
    assert can_msg[2] == 0
    assert len(can_msg) == 3

  def test_can_data_immutable(self):
    """Test CanData is immutable"""
    can_msg = CanData(0x456, b'\x05\x06', 1)

    # Should not be able to modify fields (NamedTuple is immutable)
    try:
      can_msg.address = 0x789
      assert False, "Should not be able to modify NamedTuple field"
    except AttributeError:
      pass  # Expected behavior

  def test_can_data_with_different_types(self):
    """Test CanData with various data types"""
    # Test with empty data
    can_msg = CanData(0x100, b'', 0)
    assert can_msg.dat == b''

    # Test with maximum length data (8 bytes typical for CAN)
    long_data = b'\x01\x02\x03\x04\x05\x06\x07\x08'
    can_msg = CanData(0x7FF, long_data, 2)
    assert can_msg.dat == long_data
    assert can_msg.src == 2

    # Test with extended CAN address
    can_msg = CanData(0x18DA00F1, b'\xFF', 1)
    assert can_msg.address == 0x18DA00F1

  def test_can_send_callable_type_hint(self):
    """Test CanSendCallable type hint works with functions"""
    def mock_can_send(msgs: list[CanData]) -> None:
      """Mock CAN send function"""

    # Should be compatible with CanSendCallable type
    send_func: CanSendCallable = mock_can_send

    # Should be able to call with list of CanData
    test_msgs = [
      CanData(0x123, b'\x01', 0),
      CanData(0x456, b'\x02', 0)
    ]
    send_func(test_msgs)  # Should not raise any errors

  def test_can_recv_callable_protocol(self):
    """Test CanRecvCallable protocol works with functions"""
    def mock_can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
      """Mock CAN receive function"""
      if wait_for_one:
        return [[CanData(0x123, b'\x01', 0)]]
      else:
        return [
          [CanData(0x123, b'\x01', 0), CanData(0x456, b'\x02', 0)],
          [CanData(0x789, b'\x03', 1)]
        ]

    # Should be compatible with CanRecvCallable protocol
    recv_func: CanRecvCallable = mock_can_recv

    # Test without wait_for_one
    result1 = recv_func()
    assert len(result1) == 2
    assert len(result1[0]) == 2
    assert len(result1[1]) == 1

    # Test with wait_for_one
    result2 = recv_func(wait_for_one=True)
    assert len(result2) == 1
    assert len(result2[0]) == 1

  def test_can_data_equality(self):
    """Test CanData equality comparison"""
    msg1 = CanData(0x123, b'\x01\x02', 0)
    msg2 = CanData(0x123, b'\x01\x02', 0)
    msg3 = CanData(0x456, b'\x01\x02', 0)

    # Same content should be equal
    assert msg1 == msg2

    # Different address should not be equal
    assert msg1 != msg3

    # Test with different data
    msg4 = CanData(0x123, b'\x03\x04', 0)
    assert msg1 != msg4

    # Test with different source
    msg5 = CanData(0x123, b'\x01\x02', 1)
    assert msg1 != msg5

  def test_can_data_hashing(self):
    """Test CanData can be used as dictionary key (hashable)"""
    msg1 = CanData(0x123, b'\x01\x02', 0)
    msg2 = CanData(0x456, b'\x03\x04', 1)

    # Should be able to use as dictionary keys
    can_dict = {msg1: "message1", msg2: "message2"}

    assert can_dict[msg1] == "message1"
    assert can_dict[msg2] == "message2"

    # Same content should hash to same value
    msg1_copy = CanData(0x123, b'\x01\x02', 0)
    assert can_dict[msg1_copy] == "message1"

  def test_can_data_string_representation(self):
    """Test CanData string representation"""
    can_msg = CanData(0x123, b'\x01\x02\x03', 0)

    # Should have meaningful string representation
    str_repr = str(can_msg)
    assert '0x123' in str_repr or '291' in str_repr  # Address in hex or decimal
    assert 'CanData' in str_repr

    # Should be able to convert to repr
    repr_str = repr(can_msg)
    assert 'CanData' in repr_str