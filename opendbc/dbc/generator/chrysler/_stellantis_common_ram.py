#!/usr/bin/env python3
import os

chrysler_to_ram: dict[str, dict[int, int | tuple[int, int]]] = {
  "_stellantis_common_ram_dt_generated.dbc": {
    258: 35,
    264: 37,
    268: 113,
    280: 181,
    284: 121,
    288: 123,
    320: 131,
    344: 139,
    368: 147,
    464: 464,
    500: 153,
    501: 232,
    544: 49,
    571: 177,
    559: 157,
    625: 163,
    669: 213,
    678: 250,
    720: 720,
    792: 792,
    820: 657,
  },
  "_stellantis_common_ram_hd_generated.dbc": {
    571: (570, 571),
    678: 629,
  },
}

if __name__ == "__main__":
  src = '_stellantis_common.dbc'
  chrysler_path = os.path.dirname(os.path.realpath(__file__))

  for out, addr_lookup in chrysler_to_ram.items():
    with open(os.path.join(chrysler_path, src), encoding='utf-8') as in_f, open(os.path.join(chrysler_path, out), 'w', encoding='utf-8') as out_f:
      out_f.write(f'CM_ "Generated from {src}"\n\n')

      wrote_addrs = set()
      cur_msg = []
      for line in in_f.readlines():
        cur_msg.append(line)
        if line.strip() == '':
          if not len(cur_msg):
            continue

          if not cur_msg[0].startswith(('BO_', 'VAL_')):
            continue

          # msg done
          sl = cur_msg[0].split(' ')
          addr = int(sl[1])

          new_addrs = addr_lookup.get(addr, (addr,))
          if not isinstance(new_addrs, tuple):
            new_addrs = (new_addrs,)

          for idx, new_addr in enumerate(new_addrs):
            if idx > 0:
              sl[2] = sl[2][:-1] + '_ALT:'
            sl[1] = str(new_addr)
            cur_msg[0] = ' '.join(sl)
            out_f.write(''.join(cur_msg))

          wrote_addrs.add(addr)
          cur_msg = []

          out_f.write(''.join(cur_msg))

      missing_addrs = set(addr_lookup.keys()) - wrote_addrs
      assert len(missing_addrs) == 0, f"Missing addrs from {src}: {missing_addrs}"
