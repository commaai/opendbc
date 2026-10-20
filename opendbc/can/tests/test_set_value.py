import glob
import os
import random
import unittest

from opendbc import DBC_PATH
from opendbc.can.dbc import DBC, Signal
from opendbc.can.packer import set_value


def set_value_bitwalk(msg: bytearray, sig: Signal, ival: int) -> None:
  # reference: the original per-bit walk set_value
  i = sig.lsb // 8
  bits = sig.size
  if sig.size < 64:
    ival &= (1 << sig.size) - 1
  while 0 <= i < len(msg) and bits > 0:
    shift = sig.lsb % 8 if (sig.lsb // 8) == i else 0
    size = min(bits, 8 - shift)
    mask = ((1 << size) - 1) << shift
    msg[i] &= ~mask
    msg[i] |= (ival & ((1 << size) - 1)) << shift
    bits -= size
    ival >>= size
    i = i + 1 if sig.is_little_endian else i - 1


class TestSetValue(unittest.TestCase):
  def test_matches_bitwalk(self):
    rng = random.Random(0)
    dbcs = sorted(os.path.basename(f)[:-4] for f in glob.glob(os.path.join(DBC_PATH, "*.dbc")))
    self.assertGreater(len(dbcs), 0)
    for name in dbcs:
      for msg in DBC(name).msgs.values():
        for sig in msg.sigs.values():
          vals = [0, 1, (1 << sig.size) - 1, 1 << (sig.size - 1)] + [rng.getrandbits(sig.size + 2) for _ in range(4)]
          for ival in vals:
            dat = bytearray(rng.getrandbits(8) for _ in range(msg.size))
            expected = bytearray(dat)
            set_value_bitwalk(expected, sig, ival)
            set_value(dat, sig, ival)
            self.assertEqual(expected, dat, f"{name} {msg.name} {sig.name} {ival=}")


if __name__ == "__main__":
  unittest.main()
