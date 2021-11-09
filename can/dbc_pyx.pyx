# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3
from libc.stdint cimport uint32_t, uint64_t
from libcpp.string cimport string
from libcpp.map cimport map
from .common cimport SignalParseOptions, MessageParseOptions, Msg, dbc_lookup, SignalValue, DBC, ReverseBytes

import numbers, struct

cdef class DBCParser:
  cdef:
    const DBC *dbc
    map[string, uint32_t] msg_name_to_address
    map[uint32_t, Msg] address_to_msg

  def __init__(self, dbc_name):
    self.dbc = dbc_lookup(dbc_name)
    if not self.dbc:
      raise RuntimeError(f"Can't find DBC: {dbc_name}")
    
    for i in range(self.dbc[0].msgs.size()):
      msg = self.dbc[0].msgs[i]
      name = msg.name.decode('utf8')
      self.msg_name_to_address[name] = msg.address
      self.address_to_msg[msg.address] = msg;


  def lookup_msg_id(self, msg_id):
    if not isinstance(msg_id, numbers.Number):
      msg_id = self.msg_name_to_address[msg_id.encode('utf8')]
    return msg_id

  def encode(self, msg_id, dd):
    """Encode a CAN message using the dbc.

       Inputs:
        msg_id: The message ID.
        dd: A dictionary mapping signal name to signal data.
    """
    msg_id = self.lookup_msg_id(msg_id)

    msg_def = self.address_to_msg[msg_id]
    size = msg_def.size
    result = 0
    for s in msg_def.sigs:
      ival = dd.get(s.name.decode('utf8'))
      if ival is not None:
        ival = int(round((ival - s.offset) / s.factor))
        if s.is_signed and ival < 0:
          ival = (1 << s.b2) + ival

        shift = s.b1 if s.is_little_endian else s.bo
        mask = ((1 << s.b2) - 1) << shift
        dat = (ival & ((1 << s.b2) - 1)) << shift

        if s.is_little_endian:
          mask = ReverseBytes(mask)
          dat = ReverseBytes(dat)

        result &= ~mask
        result |= dat

    result = struct.pack('>Q', result)
    return result[:size]

  def decode(self, x, arr=None, debug=False):
    """Decode a CAN message using the dbc.

       Inputs:
        x: A collection with elements (address, time, data), where address is
           the CAN address, time is the bus time, and data is the CAN data as a
           hex string.
        arr: Optional list of signals which should be decoded and returned.
        debug: True to print debugging statements.

       Returns:
        A tuple (name, data), where name is the name of the CAN message and data
        is the decoded result. If arr is None, data is a dict of properties.
        Otherwise data is a list of the same length as arr.

        Returns (None, None) if the message could not be decoded.
    """

    if arr is None:
      out = {}
    else:
      out = [None] * len(arr)

    msg = self.address_to_msg[x[0]]
    if msg is None:
      if x[0] not in self._warned_addresses:
        # print("WARNING: Unknown message address {}".format(x[0]))
        self._warned_addresses.add(x[0])
      return None, None

    name = msg.name.decode('utf8')
    if debug:
      print(name)

    st = x[2].ljust(8, b'\x00')
    le, be = None, None

    for s in msg.sigs:
      if arr is not None and s.name.decode('utf8') not in arr:
        continue

      if s.is_little_endian:
        if le is None:
          le = struct.unpack("<Q", st)[0]
        tmp = le
        shift_amount = s.b1
      else:
        if be is None:
          be = struct.unpack(">Q", st)[0]
        tmp = be
        shift_amount = s.bo

      if shift_amount < 0:
        continue

      tmp = (tmp >> shift_amount) & ((1 << s.b2) - 1)
      if s.is_signed and (tmp >> (s.b2 - 1)):
        tmp -= (1 << s.b2)

      tmp = tmp * s.factor + s.offset

      # if debug:
      #   print("%40s  %2d %2d  %7.2f " % (s.name, s.b0, s.b1, tmp))

      if arr is None:
        out[s.name.decode('utf8')] = tmp
      else:
        out[arr.index(s.name.decode('utf8'))] = tmp
    return name, out

  def get_signals(self, msg):
    msg = self.lookup_msg_id(msg)
    return [sgs.name.decode('utf8') for sgs in self.address_to_msg[msg].sigs]
