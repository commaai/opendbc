import unittest

from opendbc.safety.tests.libsafety import libsafety_py


class TestHelpers(unittest.TestCase):
  def test_get_bytes_64(self):
    # Include unaligned offsets, the end of a CAN FD payload, and high bits in every byte.
    for data in (bytes(range(64)), bytes(range(192, 256)), b"\xff" * 64):
      msg = libsafety_py.make_CANPacket(0x123, 0, data)
      for length in range(9):
        for start in range(65 - length):
          expected = int.from_bytes(data[start:start + length], "little")
          self.assertEqual(libsafety_py.libsafety.GET_BYTES_64(msg, start, length), expected)


if __name__ == "__main__":
  unittest.main()
