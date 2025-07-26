import tempfile
import os
from opendbc.can.parser import CANParser


class TestCANParser:
  def create_test_dbc(self):
    """Create a test DBC file for parser testing"""
    dbc_content = """VERSION ""

BO_ 100 TEST_MESSAGE: 8 Vector__XXX
 SG_ TEST_SIGNAL : 0|8@1+ (1,0) [0|255] "" Vector__XXX
 SG_ COUNTER_SIGNAL : 8|4@1+ (1,0) [0|15] "" Vector__XXX

BO_ 200 ALIVE_MESSAGE: 8 Vector__XXX
 SG_ DATA_SIGNAL : 0|16@1+ (0.1,0) [0|6553.5] "" Vector__XXX
"""

    with tempfile.NamedTemporaryFile(mode='w', suffix='.dbc', delete=False) as f:
      f.write(dbc_content)
      return f.name

  def test_frequency_calculation_large_dt(self):
    """Test frequency calculation when dt > 1.0 (covers lines 90-91)"""
    dbc_file = self.create_test_dbc()

    try:
      parser = CANParser(dbc_file, [(100, 0)], 0)

      # Simulate messages over long time period to trigger dt > 1.0 branch
      base_time = 1000000000  # 1 second in nanoseconds
      messages = []

      # Create messages spread over 2+ seconds to trigger dt > 1.0
      for i in range(5):
        time_ns = base_time + (i * 500_000_000)  # 500ms intervals = 2 second span
        messages.append((time_ns, [(100, b'\x00\x01\x02\x03\x04\x05\x06\x07', 0)]))

      parser.update(messages)

      # Verify frequency was calculated (should be around 2Hz based on our intervals)
      state = parser.message_states[100]
      assert state.frequency > 1e-5  # Should have calculated frequency
      assert state.timeout_threshold > 0  # Should have set timeout threshold
    finally:
      os.unlink(dbc_file)

  def test_frequency_calculation_buffer_full(self):
    """Test frequency calculation when buffer reaches max_buffer size"""
    dbc_file = self.create_test_dbc()

    try:
      parser = CANParser(dbc_file, [(100, 0)], 0)

      # Create 502 messages to exceed max_buffer of 500
      base_time = 1000000000
      messages = []

      for i in range(502):
        time_ns = base_time + (i * 10_000_000)  # 10ms intervals
        messages.append((time_ns, [(100, b'\x00\x01\x02\x03\x04\x05\x06\x07', 0)]))

      parser.update(messages)

      # Should trigger frequency calculation due to buffer size
      state = parser.message_states[100]
      assert len(state.timestamps) == 500  # Buffer should be limited to 500
      assert state.frequency > 1e-5  # Frequency should be calculated
    finally:
      os.unlink(dbc_file)

  def test_ignore_alive_branch(self):
    """Test ignore_alive branch in valid() method (covers line 104)"""
    dbc_file = self.create_test_dbc()

    try:
      parser = CANParser(dbc_file, [(100, 0)], 0)

      # Set ignore_alive to True on the message state
      state = parser.message_states[100]
      state.ignore_alive = True

      # valid() should return True immediately when ignore_alive is set
      current_time = 1000000000
      assert state.valid(current_time, False) is True
      assert state.valid(current_time, True) is True  # Should ignore bus_timeout too
    finally:
      os.unlink(dbc_file)

  def test_oversized_data_branch(self):
    """Test handling of data > 64 bytes (covers line 217)"""
    dbc_file = self.create_test_dbc()

    try:
      parser = CANParser(dbc_file, [(100, 0)], 0)

      # Create a message with data > 64 bytes
      oversized_data = b'\x00' * 65  # 65 bytes > 64 byte limit
      time_ns = 1000000000

      # This should be skipped due to length check
      updated = parser.update([(time_ns, [(100, oversized_data, 0)])])

      # Should return empty set since message was skipped
      assert 100 not in updated

      # State should not be updated
      state = parser.message_states[100]
      assert len(state.vals) == 0  # No values should be parsed
    finally:
      os.unlink(dbc_file)

  def test_no_timeout_thresholds(self):
    """Test loop when no timeout_thresholds > 0 (covers line 234->233)"""
    dbc_file = self.create_test_dbc()

    try:
      parser = CANParser(dbc_file, [(100, 0)], 0)

      # Set timeout_threshold to 0 for all states to trigger the branch
      for state in parser.message_states.values():
        state.timeout_threshold = 0

      time_ns = 1000000000
      parser.update([(time_ns, [(100, b'\x00\x01\x02\x03\x04\x05\x06\x07', 0)])])

      # Should not crash and should use default bus_timeout_threshold
      assert parser.bus_timeout is False  # Should be calculated properly
    finally:
      os.unlink(dbc_file)

  def test_unknown_address_handling(self):
    """Test handling of unknown CAN addresses"""
    dbc_file = self.create_test_dbc()

    try:
      parser = CANParser(dbc_file, [(100, 0)], 0)

      # Send message to unknown address 999
      time_ns = 1000000000
      updated = parser.update([(time_ns, [(999, b'\x00\x01\x02\x03\x04\x05\x06\x07', 0)])])

      # Should skip unknown address gracefully
      assert 999 not in updated
      assert 999 not in parser.message_states
    finally:
      os.unlink(dbc_file)

  def test_counter_validation(self):
    """Test counter validation logic"""
    dbc_content = """VERSION ""

BO_ 100 COUNTER_MESSAGE: 8 Vector__XXX
 SG_ COUNTER : 0|4@1+ (1,0) [0|15] "" Vector__XXX
"""

    with tempfile.NamedTemporaryFile(mode='w', suffix='.dbc', delete=False) as f:
      f.write(dbc_content)
      dbc_file = f.name

    try:
      # Need to manually set signal type to COUNTER (type=1)
      parser = CANParser(dbc_file, [(100, 0)], 0)
      state = parser.message_states[100]
      state.signals[0].type = 1  # Set as COUNTER type

      time_ns = 1000000000

      # Send sequence with correct counter progression
      parser.update([(time_ns, [(100, b'\x00\x00\x00\x00\x00\x00\x00\x00', 0)])])  # counter=0
      parser.update([(time_ns + 1000000, [(100, b'\x01\x00\x00\x00\x00\x00\x00\x00', 0)])])  # counter=1

      # Should have good counter
      assert state.counter_fail == 0

      # Send message with skipped counter (should increment fail count)
      parser.update([(time_ns + 2000000, [(100, b'\x05\x00\x00\x00\x00\x00\x00\x00', 0)])])  # counter=5 (skipped)

      assert state.counter_fail > 0
    finally:
      os.unlink(dbc_file)

  def test_bus_timeout_detection(self):
    """Test bus timeout detection logic"""
    dbc_file = self.create_test_dbc()

    try:
      parser = CANParser(dbc_file, [(100, 0)], 0)

      base_time = 1000000000

      # Send initial message
      parser.update([(base_time, [(100, b'\x00\x01\x02\x03\x04\x05\x06\x07', 0)])])

      # Send empty frame (no messages for this bus)
      empty_time = base_time + 600_000_000  # 600ms later, should trigger timeout
      parser.update([(empty_time, [])])  # Empty frames list

      # Should detect bus timeout
      assert parser.bus_timeout is True
    finally:
      os.unlink(dbc_file)