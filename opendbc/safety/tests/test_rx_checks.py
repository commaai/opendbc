import unittest

from opendbc.safety.tests.common import make_msg
from opendbc.safety.tests.libsafety import libsafety_py


class TestRxChecks(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety

  def test_field_extraction(self):
    # Exercise every single-byte value, including fields ending at the last CAN FD byte.
    for value in range(256):
      msg = make_msg(0, 0x123, dat=bytes([value]) * 64)
      for shift in range(8):
        for width in range(1, 9 - shift):
          mask = (1 << width) - 1
          self.assertEqual(self.safety.get_rx_msg_field(msg, 63, shift, mask), (value >> shift) & mask)

    # Little-endian fields spanning two bytes, with independently varying low and high bytes.
    for value in range(65536):
      msg = make_msg(0, 0x123, dat=bytes(62) + value.to_bytes(2, "little"))
      self.assertEqual(self.safety.get_rx_msg_field(msg, 62, 0, 0xFFFF), value)
      self.assertEqual(self.safety.get_rx_msg_field(msg, 62, 4, 0xFFF), value >> 4)

  def test_missing_metadata_fails_closed(self):
    msg = make_msg(0, 0x123)
    for descriptor_present in (False, True):
      for checksum, counter, max_counter, quality in ((False, True, 0, True), (True, False, 0, True),
                                                     (True, False, 15, True), (True, True, 0, False)):
        with self.subTest(descriptor_present=descriptor_present, checksum=checksum, counter=counter,
                          max_counter=max_counter, quality=quality):
          self.safety.set_controls_allowed(True)
          self.assertFalse(self.safety.rx_check_missing_metadata(msg, descriptor_present, checksum, counter, max_counter, quality))
          self.assertFalse(self.safety.get_controls_allowed())

      self.safety.set_controls_allowed(True)
      self.assertTrue(self.safety.rx_check_missing_metadata(msg, descriptor_present, True, True, 0, True))
      self.assertTrue(self.safety.get_controls_allowed())

  def test_metadata_requires_exact_message_match(self):
    # Missing required metadata rejects the configured message, but is never used for other traffic.
    for msg in (make_msg(0, 0x124), make_msg(1, 0x123), make_msg(0, 0x123, length=7), make_msg(0, 0x123, length=12)):
      self.safety.set_controls_allowed(True)
      self.assertTrue(self.safety.rx_check_missing_metadata(msg, True, False, False, 15, False))
      self.assertTrue(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
