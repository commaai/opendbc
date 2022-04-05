#!/usr/bin/env python3
import re
import os
import sys
import numbers
from collections import namedtuple, defaultdict

def int_or_float(s):
  # return number, trying to maintain int format
  if s.isdigit():
    return int(s, 10)
  else:
    return float(s)


DBCSignal = namedtuple("DBCSignal", ["name", "start_bit", "msb", "lsb", "size", "is_little_endian", "is_signed",
                                     "factor", "offset", "tmin", "tmax", "units"])


class dbc():
  def __init__(self, fn):
    self.name, _ = os.path.splitext(os.path.basename(fn))
    with open(fn, encoding="utf-8") as f:
      self.txt = f.readlines()
    self._warned_addresses = set()

    # regexps from https://github.com/ebroecker/canmatrix/blob/master/canmatrix/importdbc.py
    bo_regexp = re.compile(r"^BO\_ (\w+) (\w+) *: (\w+) (\w+)")
    sg_regexp = re.compile(r"^SG\_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*)")
    sgm_regexp = re.compile(r"^SG\_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*)")
    val_regexp = re.compile(r"VAL\_ (\w+) (\w+) (\s*[-+]?[0-9]+\s+\".+?\"[^;]*)")

    # A dictionary which maps message ids to tuples ((name, size), signals).
    #   name is the ASCII name of the message.
    #   size is the size of the message in bytes.
    #   signals is a list signals contained in the message.
    # signals is a list of DBCSignal in order of increasing start_bit.
    self.msgs = {}

    # A dictionary which maps message ids to a list of tuples (signal name, definition value pairs)
    self.def_vals = defaultdict(list)

    # used to find big endian LSB from MSB and size
    be_bits = [(j + i*8) for i in range(64) for j in range(7, -1, -1)]

    for l in self.txt:
      l = l.strip()

      if l.startswith("BO_ "):
        # new group
        dat = bo_regexp.match(l)

        if dat is None:
          print("bad BO {0}".format(l))

        name = dat.group(2)
        size = int(dat.group(3))
        ids = int(dat.group(1), 0)  # could be hex
        if ids in self.msgs:
          sys.exit("Duplicate address detected %d %s" % (ids, self.name))

        self.msgs[ids] = ((name, size), [])

      if l.startswith("SG_ "):
        # new signal
        dat = sg_regexp.match(l)
        go = 0
        if dat is None:
          dat = sgm_regexp.match(l)
          go = 1

        if dat is None:
          print("bad SG {0}".format(l))

        sgname = dat.group(1)
        start_bit = int(dat.group(go + 2))
        signal_size = int(dat.group(go + 3))
        is_little_endian = int(dat.group(go + 4)) == 1
        is_signed = dat.group(go + 5) == '-'
        factor = int_or_float(dat.group(go + 6))
        offset = int_or_float(dat.group(go + 7))
        tmin = int_or_float(dat.group(go + 8))
        tmax = int_or_float(dat.group(go + 9))
        units = dat.group(go + 10)

        if is_little_endian:
          lsb = start_bit
          msb = start_bit + signal_size - 1
        else:
          lsb = be_bits[be_bits.index(start_bit) + signal_size - 1]
          msb = start_bit

        self.msgs[ids][1].append(
          DBCSignal(sgname, start_bit, msb, lsb, signal_size, is_little_endian,
                    is_signed, factor, offset, tmin, tmax, units))

        assert lsb < (64*8) and msb < (64*8), f"Signal out of bounds: {msb=} {lsb=}"

      if l.startswith("VAL_ "):
        # new signal value/definition
        dat = val_regexp.match(l)

        if dat is None:
          print("bad VAL {0}".format(l))

        ids = int(dat.group(1), 0)  # could be hex
        sgname = dat.group(2)
        defvals = dat.group(3)

        defvals = defvals.replace("?", r"\?")  # escape sequence in C++
        defvals = defvals.split('"')[:-1]

        # convert strings to UPPER_CASE_WITH_UNDERSCORES
        defvals[1::2] = [d.strip().upper().replace(" ", "_") for d in defvals[1::2]]
        defvals = '"' + "".join(str(i) for i in defvals) + '"'

        self.def_vals[ids].append((sgname, defvals))

    for msg in self.msgs.values():
      msg[1].sort(key=lambda x: x.start_bit)

    self.msg_name_to_address = {}
    for address, m in self.msgs.items():
      name = m[0][0]
      self.msg_name_to_address[name] = address

  def lookup_msg_id(self, msg_id):
    if not isinstance(msg_id, numbers.Number):
      msg_id = self.msg_name_to_address[msg_id]
    return msg_id

  def get_signals(self, msg):
    msg = self.lookup_msg_id(msg)
    return [sgs.name for sgs in self.msgs[msg][1]]
