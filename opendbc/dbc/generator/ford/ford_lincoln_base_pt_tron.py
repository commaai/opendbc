#!/usr/bin/env python3
import os

from opendbc import DBC_PATH

# Messages that are 16 bytes on TRON platforms, seen on a 2026 F-150.
# The signals are where the base DBC has them, in the first 8 bytes
TRON_ADDRS = {
  0x05A, 0x05C, 0x076, 0x077, 0x07D, 0x082, 0x084, 0x165, 0x166, 0x167, 0x168, 0x171, 0x178, 0x186, 0x187, 0x18A,
  0x200, 0x202, 0x204, 0x205, 0x213, 0x216, 0x217, 0x242, 0x25B, 0x331, 0x332, 0x333, 0x38D, 0x3A1, 0x3A2, 0x3A3,
  0x3A6, 0x3A7, 0x3A8, 0x3A9, 0x3AA, 0x3AB, 0x3AE, 0x3B3, 0x3BA, 0x3C3, 0x3CA, 0x3CC, 0x3D6, 0x3D7, 0x3D8, 0x3D9,
  0x3EB, 0x3F2, 0x40A, 0x414, 0x415, 0x416, 0x41E, 0x420, 0x42C, 0x43F, 0x44C, 0x451, 0x4B0, 0x4E0, 0x4E1, 0x4E2,
  0x4E3, 0x4E4, 0x4E5, 0x4E6, 0x4E7,
}


def generate():
  src = 'ford_lincoln_base_pt.dbc'
  parts = [f'CM_ "Generated from {src}";\n\n']

  wrote_addrs = set()
  with open(os.path.join(DBC_PATH, src), encoding='utf-8') as in_f:
    for line in in_f.readlines():
      if line.startswith('BO_ '):
        sl = line.split(' ')
        addr = int(sl[1])
        if addr in TRON_ADDRS:
          assert sl[3] == '8', f"Unexpected size for {addr:#x} in {src}: {sl[3]}"
          sl[3] = '16'
          line = ' '.join(sl)
          wrote_addrs.add(addr)
      parts.append(line)

  missing_addrs = TRON_ADDRS - wrote_addrs
  assert len(missing_addrs) == 0, f"Missing addrs from {src}: {missing_addrs}"

  return {'ford_lincoln_base_pt_tron.dbc': ''.join(parts)}
