# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector

from .common cimport CANParser as cpp_CANParser
from .common cimport dbc_lookup, SignalValue, DBC

from collections import defaultdict


cdef class CANParser:
  cdef cpp_CANParser *can
  cdef readonly:
    dict vl
    dict vl_all
    dict ts_nanos
    string dbc_name

  def __init__(self, dbc_name, messages, bus=0):
    self.dbc_name = dbc_name
    self.vl = {}
    self.vl_all = {}
    self.ts_nanos = {}

    cdef vector[pair[string, int]] message_v
    for name_or_address, freq in messages:
      message_v.push_back((str(name_or_address), freq))
    self.can = new cpp_CANParser(bus, dbc_name, message_v)

    # build value lookup table
    for msg in self.can.messages():
      msg_name = <unicode>msg.name
      self.vl[msg_name] = self.vl[msg.address] = {}
      self.vl_all[msg_name] = self.vl_all[msg.address] = {}
      self.ts_nanos[msg_name] = self.ts_nanos[msg.address] = {}
      for sig in msg.sigs:
        sig_name = <unicode>sig.name
        self.vl[msg.address][sig_name] = 0
        self.ts_nanos[msg.address][sig_name] = 0
        self.vl_all[msg.address][sig_name] = []

  def update_strings(self, strings, sendcan=False):
    for v in self.vl_all.values():
      for l in v.values():  # no-cython-lint
        l.clear()

    cdef vector[SignalValue] new_vals
    updated_addrs = set()
    self.can.update_strings(strings, new_vals, sendcan)
    for cv in new_vals:
      # Cast char * directly to unicode
      cv_name = <unicode>cv.name
      self.vl[cv.address][cv_name] = cv.value
      self.vl_all[cv.address][cv_name] = cv.all_values
      self.ts_nanos[cv.address][cv_name] = cv.ts_nanos
      updated_addrs.add(cv.address)

    return updated_addrs

  @property
  def can_valid(self):
    return self.can.can_valid

  @property
  def bus_timeout(self):
    return self.can.bus_timeout


cdef class CANDefine():
  cdef:
    const DBC *dbc

  cdef public:
    dict dv
    string dbc_name

  def __init__(self, dbc_name):
    self.dbc_name = dbc_name
    self.dbc = dbc_lookup(dbc_name)
    if not self.dbc:
      raise RuntimeError(f"Can't find DBC: '{dbc_name}'")

    address_to_msg_name = {}

    for i in range(self.dbc[0].msgs.size()):
      msg = self.dbc[0].msgs[i]
      name = msg.name.decode("utf8")
      address = msg.address
      address_to_msg_name[address] = name

    dv = defaultdict(dict)

    for i in range(self.dbc[0].vals.size()):
      val = self.dbc[0].vals[i]

      sgname = val.name.decode("utf8")
      def_val = val.def_val.decode("utf8")
      address = val.address
      msgname = address_to_msg_name[address]

      # separate definition/value pairs
      def_val = def_val.split()
      values = [int(v) for v in def_val[::2]]
      defs = def_val[1::2]

      # two ways to lookup: address or msg name
      dv[address][sgname] = dict(zip(values, defs))
      dv[msgname][sgname] = dv[address][sgname]

    self.dv = dict(dv)
