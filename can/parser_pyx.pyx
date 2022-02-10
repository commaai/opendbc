# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.unordered_set cimport unordered_set
from libc.stdint cimport uint32_t, uint64_t, uint16_t
from libcpp.map cimport map
from libcpp cimport bool

from .common cimport CANParser as cpp_CANParser
from .common cimport SignalParseOptions, MessageParseOptions, dbc_lookup, SignalValue, DBC

import os
import numbers
from collections import defaultdict

cdef int CAN_INVALID_CNT = 5

cdef class CANParserField:
  cdef:
    dict msg_name_to_address

  cdef public:
    dict dat

  def __init__(self):
    self.dat = {}

  def __getitem__(self, key):
    if not isinstance(key, int):
      key = self.msg_name_to_address[key]
    return self.dat[key]

cdef class CANParser:
  cdef:
    cpp_CANParser *can
    const DBC *dbc
    dict msg_name_to_address
    map[uint32_t, string] address_to_msg_name
    vector[SignalValue] can_values
    bool test_mode_enabled

  cdef readonly:
    string dbc_name
    bool can_valid
    int can_invalid_cnt
    CANParserField vl
    CANParserField ts
    CANParserField updated

  def __init__(self, dbc_name, signals, checks=None, bus=0, enforce_checks=True):
    if checks is None:
      checks = []
    self.can_valid = True
    self.dbc_name = dbc_name
    self.dbc = dbc_lookup(dbc_name)
    if not self.dbc:
      raise RuntimeError(f"Can't find DBC: {dbc_name}")
    self.vl = CANParserField()
    self.ts = CANParserField()
    self.updated = CANParserField()
    self.msg_name_to_address = {}

    self.can_invalid_cnt = CAN_INVALID_CNT

    cdef int i
    cdef int num_msgs = self.dbc[0].num_msgs
    for i in range(num_msgs):
      msg = self.dbc[0].msgs[i]
      name = msg.name.decode('utf8')

      self.msg_name_to_address[<unicode>name] = msg.address
      self.address_to_msg_name[msg.address] = name

      self.vl.dat[msg.address] = {}
      self.ts.dat[msg.address] = {}
      self.updated.dat[msg.address] = {}

    self.vl.msg_name_to_address = self.msg_name_to_address
    self.ts.msg_name_to_address = self.msg_name_to_address
    self.updated.msg_name_to_address = self.msg_name_to_address

    # Convert message names into addresses
    for i in range(len(signals)):
      s = signals[i]
      if not isinstance(s[1], numbers.Number):
        s = (s[0], self.msg_name_to_address[s[1]])
        signals[i] = s

    for i in range(len(checks)):
      c = checks[i]
      if not isinstance(c[0], numbers.Number):
        c = (self.msg_name_to_address[c[0]], c[1])
        checks[i] = c

    if enforce_checks:
      checked_addrs = {c[0] for c in checks}
      signal_addrs = {s[1] for s in signals}
      unchecked = signal_addrs - checked_addrs
      if len(unchecked):
        err_msg = ', '.join(f"{self.address_to_msg_name[addr].decode()} ({hex(addr)})" for addr in unchecked)
        raise RuntimeError(f"Unchecked addrs: {err_msg}")

    cdef vector[SignalParseOptions] signal_options_v
    cdef SignalParseOptions spo
    for sig_name, sig_address in signals:
      spo.address = sig_address
      spo.name = sig_name
      signal_options_v.push_back(spo)

    message_options = dict((address, 0) for _, address in signals)
    message_options.update(dict(checks))

    cdef vector[MessageParseOptions] message_options_v
    cdef MessageParseOptions mpo
    for msg_address, freq in message_options.items():
      mpo.address = msg_address
      mpo.check_frequency = freq
      message_options_v.push_back(mpo)

    self.can = new cpp_CANParser(bus, dbc_name, message_options_v, signal_options_v)
    self.update_vl(init=True)

  cdef unordered_set[uint32_t] update_vl(self, init=False):
    cdef string sig_name
    cdef unordered_set[uint32_t] updated_val

    can_values = self.can.update_vl()
    valid = self.can.can_valid

    # Update invalid flag
    self.can_invalid_cnt += 1
    if valid:
      self.can_invalid_cnt = 0
    self.can_valid = self.can_invalid_cnt < CAN_INVALID_CNT

    for cv in can_values:
      cv_name = <unicode>cv.name

      self.updated.dat[cv.address][cv_name] = cv.updated_values
      if init or len(cv.updated_values) > 0:
        self.vl.dat[cv.address][cv_name] = cv.value
        self.ts.dat[cv.address][cv_name] = cv.ts
        if not init:
          updated_val.insert(cv.address)

    return updated_val

  def update_string(self, dat, sendcan=False):
    self.can.update_string(dat, sendcan)
    return self.update_vl()

  def update_strings(self, strings, sendcan=False):
    for s in strings:
      self.can.update_string(s, sendcan)

    return self.update_vl()

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

    num_vals = self.dbc[0].num_vals

    address_to_msg_name = {}

    num_msgs = self.dbc[0].num_msgs
    for i in range(num_msgs):
      msg = self.dbc[0].msgs[i]
      name = msg.name.decode('utf8')
      address = msg.address
      address_to_msg_name[address] = name

    dv = defaultdict(dict)

    for i in range(num_vals):
      val = self.dbc[0].vals[i]

      sgname = val.name.decode('utf8')
      def_val = val.def_val.decode('utf8')
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
