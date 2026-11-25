import unittest
from collections import deque
from unittest.mock import Mock, patch

from opendbc.car.can_definitions import CanData
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
from opendbc.car.uds import CanClient, IsoTpMessage, UdsClient


class TestIsoTpMessage(unittest.TestCase):
  def setUp(self):
    self.can_send = Mock()
    self.can_recv = Mock(return_value=[])

  def message(self, sub_addr=None, request=b"\x22\xf1\x90", single_frame_mode=False):
    client = CanClient(self.can_send, self.can_recv, 0x700, 0x708, 0, sub_addr=sub_addr)
    msg = IsoTpMessage(client, timeout=0, single_frame_mode=single_frame_mode)
    msg.send(request)
    self.can_send.reset_mock()
    return msg

  def receive(self, msg, *frames):
    sub_addr = msg._can_client.sub_addr
    prefix = b"" if sub_addr is None else bytes([sub_addr])
    self.can_recv.side_effect = [[(0x708, prefix + frame, 0) for frame in frames], []]
    try:
      return msg.recv()
    finally:
      self.can_recv.side_effect = None
      self.can_recv.return_value = []

  def test_single_frame_lengths(self):
    for sub_addr in (None, 0):
      max_len = 8 if sub_addr is None else 7
      for size in range(1, max_len):
        payload = bytes(range(size))
        for frame_len in range(size + 1, max_len + 1):
          with self.subTest(sub_addr=sub_addr, size=size, frame_len=frame_len):
            msg = self.message(sub_addr)
            frame = (bytes([size]) + payload).ljust(frame_len, b"\xcc")
            self.assertEqual(self.receive(msg, frame), (payload, False))
            self.can_send.assert_not_called()

  def test_invalid_single_and_first_frames(self):
    for sub_addr in (None, 0):
      max_len = 8 if sub_addr is None else 7
      frames = ([b""] if sub_addr is not None else []) + [b"\x00", b"\x00" * max_len, bytes([max_len - 1]) + b"a",
                bytes([max_len]) + b"a" * (max_len - 1), bytes([max_len - 1]) + b"a" * max_len,
                b"\x10", bytes([0x10, max_len]) + b"a", bytes([0x10, max_len - 1]) + b"a" * (max_len - 2)]
      for frame in frames:
        with self.subTest(sub_addr=sub_addr, frame=frame):
          msg = self.message(sub_addr)
          state = vars(msg).copy()
          with self.assertRaises(AssertionError):
            self.receive(msg, frame)
          self.assertEqual(vars(msg), state)
          self.can_send.assert_not_called()

  def test_can_fd_single_frames(self):
    for sub_addr in (None, 0):
      max_len = 64 if sub_addr is None else 63
      for size in (8, max_len - 2):
        with self.subTest(sub_addr=sub_addr, size=size):
          msg = self.message(sub_addr)
          payload = bytes(range(size))
          frame = (bytes([0, size]) + payload).ljust(max_len, b"\xcc")
          self.assertEqual(self.receive(msg, frame), (payload, False))

      for frame in (bytes([0, max_len - 2]) + b"a" * 10, bytes([0, 0]) + b"a" * (max_len - 2),
                    bytes([0, max_len - 1]) + b"a" * (max_len - 2)):
        with self.subTest(sub_addr=sub_addr, frame=frame):
          msg = self.message(sub_addr)
          state = vars(msg).copy()
          with self.assertRaises(AssertionError):
            self.receive(msg, frame)
          self.assertEqual(vars(msg), state)

  def test_consecutive_frame_requires_first_frame(self):
    for sub_addr in (None, 0):
      with self.subTest(sub_addr=sub_addr):
        msg = self.message(sub_addr)
        state = vars(msg).copy()
        with self.assertRaisesRegex(AssertionError, "no active frame"):
          self.receive(msg, b"\x21a")
        self.assertEqual(vars(msg), state)

  def test_invalid_consecutive_frames_preserve_state(self):
    for sub_addr in (None, 0):
      max_len = 8 if sub_addr is None else 7
      for frame in (b"\x21", b"\x21a", b"\x21" + b"a" * max_len, b"\x22" + b"a" * (max_len - 1)):
        with self.subTest(sub_addr=sub_addr, frame=frame):
          msg = self.message(sub_addr)
          self.assertEqual(self.receive(msg, bytes([0x10, 30]) + b"a" * (max_len - 2)), (None, True))
          state = vars(msg).copy()
          self.can_send.reset_mock()
          with self.assertRaises(AssertionError):
            self.receive(msg, frame)
          self.assertEqual(vars(msg), state)
          self.can_send.assert_not_called()
          self.assertEqual(self.receive(msg, b"\x21" + b"b" * (max_len - 1)), (None, True))

  def test_truncated_final_consecutive_frame(self):
    for sub_addr in (None, 0):
      with self.subTest(sub_addr=sub_addr):
        msg = self.message(sub_addr)
        self.receive(msg, bytes([0x10, msg.max_len]) + b"a" * (msg.max_len - 2))
        state = vars(msg).copy()
        with self.assertRaises(AssertionError):
          self.receive(msg, b"\x21b")
        self.assertEqual(vars(msg), state)
        self.assertEqual(self.receive(msg, b"\x21bc"), (b"a" * (msg.max_len - 2) + b"bc", False))

  def test_multiframe_lengths_and_sequence_wrap(self):
    for sub_addr in (None, 0):
      for single_frame_mode in (False, True):
        for padded in (False, True):
          for size in (8, 12, 13, 14, 120, 4095):
            with self.subTest(sub_addr=sub_addr, single_frame_mode=single_frame_mode, padded=padded, size=size):
              msg = self.message(sub_addr, single_frame_mode=single_frame_mode)
              payload = bytes(i % 256 for i in range(size))
              first_size = msg.max_len - 2
              frames = [bytes([0x10 | (size >> 8), size & 0xff]) + payload[:first_size]]
              for index, offset in enumerate(range(first_size, size, msg.max_len - 1), start=1):
                frame = bytes([0x20 | (index & 0xf)]) + payload[offset:offset + msg.max_len - 1]
                frames.append(frame.ljust(msg.max_len, b"\xcc") if padded else frame)
              self.assertEqual(self.receive(msg, *frames), (payload, False))
              expected_fc = len(frames) - 1 if single_frame_mode else 1
              self.assertEqual(self.can_send.call_count, expected_fc)

  def test_short_flow_control(self):
    for sub_addr in (None, 0):
      max_len = 8 if sub_addr is None else 7
      for frame_len in range(3, max_len + 1):
        with self.subTest(sub_addr=sub_addr, frame_len=frame_len):
          msg = self.message(sub_addr, request=bytes(range(24)))
          self.assertEqual(self.receive(msg, b"\x30\x00\x00".ljust(frame_len, b"\xcc")), (None, False))
          self.assertTrue(msg.tx_done)
          self.assertEqual(self.can_send.call_count, 3 if sub_addr is None else 4)

  def test_invalid_flow_control_lengths(self):
    for sub_addr in (None, 0):
      max_len = 8 if sub_addr is None else 7
      for frame in (b"\x30", b"\x30\x00", b"\x31", b"\x31\x00", b"\x30\x00\x00" + b"a" * (max_len - 2)):
        with self.subTest(sub_addr=sub_addr, frame=frame):
          msg = self.message(sub_addr, request=bytes(range(24)))
          state = vars(msg).copy()
          with self.assertRaises(AssertionError):
            self.receive(msg, frame)
          self.assertEqual(vars(msg), state)
          self.can_send.assert_not_called()

  def test_send_discards_stale_response(self):
    for buffered in (False, True):
      with self.subTest(buffered=buffered):
        client = CanClient(self.can_send, self.can_recv, 0x700, 0x708, 0)
        stale = b"\x03\x62\xf1\x90"
        if buffered:
          client.rx_buff.append(stale)
        else:
          self.can_recv.side_effect = [[(0x708, stale, 0)], []]
        msg = IsoTpMessage(client, timeout=0)
        msg.send(b"\x22\xf1\x90")
        self.assertEqual(msg.recv(), (None, False))
        self.assertEqual(list(client.rx_buff), [])
        self.can_recv.side_effect = None


class TestCanClient(unittest.TestCase):
  def test_functional_response_stays_on_requested_bus(self):
    for tx_addr, wrong_rx, correct_rx, physical_tx in ((0x7df, 0x7e8, 0x7e9, 0x7e1),
                                                     (0x18db33f1, 0x18daf110, 0x18daf111, 0x18da11f1)):
      with self.subTest(tx_addr=tx_addr):
        can_recv = Mock(return_value=[(wrong_rx, b"\x01a", 0)])
        can_send = Mock()
        client = CanClient(can_send, can_recv, tx_addr, None, 1)
        self.assertEqual(list(client.recv()), [])
        self.assertEqual((client.tx_addr, client.rx_addr), (tx_addr, None))
        can_recv.return_value = [(correct_rx, b"\x01b", 1)]
        self.assertEqual(list(client.recv()), [b"\x01b"])
        client.send([b"\x30\x00\x00"])
        can_send.assert_called_once_with(physical_tx, b"\x30\x00\x00", 1)

  def test_empty_functional_response_does_not_select_ecu(self):
    for tx_addr, rx_addr in ((0x7df, 0x7e8), (0x18db33f1, 0x18daf110)):
      with self.subTest(tx_addr=tx_addr):
        client = CanClient(Mock(), Mock(return_value=[(rx_addr, b"", 0)]), tx_addr, None, 0)
        self.assertEqual(list(client.recv()), [])
        self.assertEqual((client.tx_addr, client.rx_addr), (tx_addr, None))


class TestIsoTpParallelQuery(unittest.TestCase):
  def query(self, addrs, packets, request=None, response=None):
    batches = deque(packets)

    def can_recv(wait_for_one=False):
      return [batches.popleft()] if wait_for_one and batches else []

    return IsoTpParallelQuery(Mock(), can_recv, 0, addrs, request or [b"\x22\xf1\x90"], response or [b"\x62\xf1\x90"])

  def test_multi_request_discards_buffered_duplicate(self):
    query = self.query([0x700], [[CanData(0x708, b"\x02\x50\x03", 0), CanData(0x708, b"\x02\x50\x03", 0)],
                                [CanData(0x708, b"\x04\x62\xf1\x90a", 0)]],
                       request=[b"\x10\x03", b"\x22\xf1\x90"], response=[b"\x50\x03", b"\x62\xf1\x90"])
    self.assertEqual(query.get_data(0.1), {(0x700, None): b"a"})

  def test_truncated_response_does_not_hide_healthy_ecu(self):
    query = self.query([0x700, 0x710], [[CanData(0x708, b"\x07\x62\xf1\x90a", 0),
                                      CanData(0x718, b"\x04\x62\xf1\x90b", 0)]])
    with patch("opendbc.car.isotp_parallel_query.carlog.exception") as log_exception:
      self.assertEqual(query.get_data(0.1), {(0x710, None): b"b"})
    log_exception.assert_called_once()

  def test_empty_frame_does_not_hide_subaddressed_ecus(self):
    query = self.query([(0x700, 0), (0x700, 1)], [[CanData(0x708, b"", 0),
                                               CanData(0x708, b"\x00\x04\x62\xf1\x90a", 0),
                                               CanData(0x708, b"\x01\x04\x62\xf1\x90b", 0)]])
    self.assertEqual(query.get_data(0.1), {(0x700, 0): b"a", (0x700, 1): b"b"})

  def test_completed_response_cannot_be_overwritten(self):
    query = self.query([0x700, 0x710], [[CanData(0x708, b"\x04\x62\xf1\x90a", 0)],
                                      [CanData(0x708, b"\x04\x62\xf1\x90c", 0), CanData(0x718, b"\x04\x62\xf1\x90b", 0)]])
    self.assertEqual(query.get_data(0.1), {(0x700, None): b"a", (0x710, None): b"b"})
    self.assertEqual(query.msg_buffer[0x708], [])

  def test_failed_response_cannot_be_replaced(self):
    query = self.query([0x700, 0x710], [[CanData(0x708, b"\x08" + b"a" * 7, 0)],
                                      [CanData(0x708, b"\x04\x62\xf1\x90a", 0), CanData(0x718, b"\x04\x62\xf1\x90b", 0)]])
    with patch("opendbc.car.isotp_parallel_query.carlog.exception") as log_exception:
      self.assertEqual(query.get_data(0.1), {(0x710, None): b"b"})
    log_exception.assert_called_once()


class TestUdsClient(unittest.TestCase):
  def test_pending_and_positive_responses_in_one_batch(self):
    replies = []

    def can_send(addr, dat, bus, timeout):
      replies.extend([(0x708, b"\x03\x7f\x22\x78", 0), (0x708, b"\x05\x62\xf1\x90ab", 0)])

    def can_recv():
      packets = replies.copy()
      replies.clear()
      return packets

    panda = Mock(can_send=Mock(side_effect=can_send), can_recv=Mock(side_effect=can_recv))
    client = UdsClient(panda, 0x700, timeout=0.1)
    self.assertEqual(client.read_data_by_identifier(0xf190), b"ab")
    panda.can_send.assert_called_once_with(0x700, b"\x03\x22\xf1\x90\x00\x00\x00\x00", 0, timeout=1000)


if __name__ == "__main__":
  unittest.main()
