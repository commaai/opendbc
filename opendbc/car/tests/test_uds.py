import unittest
from unittest import mock

from opendbc.car import uds


class TestIsoTpSeparationTime(unittest.TestCase):
  def _make_isotp_msg(self, separation_time: float = 0) -> uds.IsoTpMessage:
    can_client = uds.CanClient(lambda *args: None, list, 0x7E0, 0x7E8, 0)
    return uds.IsoTpMessage(can_client, separation_time=separation_time)

  def test_tx_flow_control_separation_time(self):
    # ISO 15765-2 STmin: 0x00-0x7F is 0-127 ms, 0xF1-0xF9 is 100-900 us
    for separation_time, st_min in [(0, 0x00), (0.001, 0x01), (0.01, 0x0A), (0.127, 0x7F),
                                    (0.0001, 0xF1), (0.0002, 0xF2), (0.0003, 0xF3), (0.0004, 0xF4), (0.0005, 0xF5),
                                    (0.0006, 0xF6), (0.0007, 0xF7), (0.0008, 0xF8), (0.0009, 0xF9)]:
      with self.subTest(separation_time=separation_time):
        isotp_msg = self._make_isotp_msg(separation_time)
        self.assertEqual(isotp_msg.flow_control_msg[2], st_min)

  def test_rx_flow_control_separation_time(self):
    # reserved values must be treated as the longest valid STmin, 127 ms
    for st_min, delay in [(0x00, 0), (0x01, 0.001), (0x0A, 0.01), (0x7F, 0.127),
                          (0xF1, 0.0001), (0xF5, 0.0005), (0xF9, 0.0009),
                          (0x80, 0.127), (0xF0, 0.127), (0xFA, 0.127), (0xFF, 0.127)]:
      with self.subTest(st_min=hex(st_min)):
        isotp_msg = self._make_isotp_msg()
        isotp_msg.send(bytes(range(20)))
        with mock.patch.object(isotp_msg._can_client, "send") as send:
          isotp_msg._isotp_rx_next(bytes([0x30, 0x00, st_min, 0, 0, 0, 0, 0]))
        self.assertEqual(len(send.call_args.args[0]), 2)
        self.assertAlmostEqual(send.call_args.kwargs["delay"], delay)


if __name__ == "__main__":
  unittest.main()
