#!/usr/bin/env python3
import os
from pathlib import Path

CHRYSLER_TO_RAM_ADDR = {
  258: 35,
  284: 121,
  320: 131,
  344: 139,
  464: 464,
  500: 153,
  501: 232,
  544: 49,
  571: 177,
  559: 157,
  678: 250,
  720: 720,
  792: 792,
  820: 657,
}

if __name__ == "__main__":
  src = '_stellantis_common.dbc'
  out = Path(__file__).stem + '.dbc'
  chrysler_path = os.path.dirname(os.path.realpath(__file__))

  with open(os.path.join(chrysler_path, src)) as in_f, open(os.path.join(chrysler_path, out), 'w') as out_f:
    out_f.write(f'CM_ "Generated from {src}"\n\n')

    wrote_addrs = set()
    for line in in_f.readlines():
      if line.startswith('BO_'):
        sl = line.split(' ')
        addr = int(sl[1])
        wrote_addrs.add(addr)

        sl[1] = str(CHRYSLER_TO_RAM_ADDR.get(addr, addr))
        line = ' '.join(sl)
      out_f.write(line)

    missing_addrs = set(CHRYSLER_TO_RAM_ADDR.keys()) - wrote_addrs
    assert len(missing_addrs) == 0, f"Missing addrs from {src}: {missing_addrs}"
