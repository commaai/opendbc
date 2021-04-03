# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

from libcpp.string cimport string
from libcpp.vector cimport vector

import threading
from opendbc.can.process_dbc import process
from .common cimport dbc_lookup, SignalValue, DBC, Msg, SignalType, Signal, Val, dbc_register

cdef get_sigs(address, checksum_type, sigs):
  cdef vector[Signal] ret
  cdef Signal s
  for sig in sigs:
    b1 = sig.start_bit if sig.is_little_endian else ((sig.start_bit//8)*8  + (-sig.start_bit-1) % 8)
    s.name = sig.name
    s.b1 = b1
    s.b2 = sig.size
    s.bo = 64 - (b1 + sig.size)
    s.is_signed = sig.is_signed
    s.factor = sig.factor
    s.offset = sig.offset
    s.is_little_endian = sig.is_little_endian
    if checksum_type == "honda" and sig.name == "CHECKSUM":
      s.type = SignalType.HONDA_CHECKSUM
    elif checksum_type == "honda" and sig.name == "COUNTER":
      s.type = SignalType.HONDA_COUNTER
    elif checksum_type == "toyota" and sig.name == "CHECKSUM":
      s.type = SignalType.TOYOTA_CHECKSUM
    elif checksum_type == "volkswagen" and sig.name == "CHECKSUM":
      s.type = SignalType.VOLKSWAGEN_CHECKSUM
    elif checksum_type == "volkswagen" and sig.name == "COUNTER":
      s.type = SignalType.VOLKSWAGEN_COUNTER
    elif checksum_type == "subaru" and sig.name == "CHECKSUM":
      s.type = SignalType.SUBARU_CHECKSUM
    elif checksum_type == "chrysler" and sig.name == "CHECKSUM":
      s.type = SignalType.CHRYSLER_CHECKSUM
    elif address in [512, 513] and sig.name == "CHECKSUM_PEDAL":
      s.type = SignalType.PEDAL_CHECKSUM
    elif address in [512, 513] and sig.name == "COUNTER_PEDAL":
      s.type = SignalType.PEDAL_COUNTER
    else:
      s.type = SignalType.DEFAULT

    ret.push_back(s)
  return ret

cdef register_dbc(name, checksum_type, msgs, def_vals):
  sig_map = {}
  cdef DBC dbc
  dbc.name = name
  cdef Msg msg
  for address, msg_name, msg_size, sigs in msgs:
    msg.name = msg_name
    msg.address = address
    msg.size = msg_size
    msg.sigs = get_sigs(address, checksum_type, sigs)
    msg.num_sigs = msg.sigs.size()
    dbc.msgs.push_back(msg);
    sig_map[address] = msg.sigs

  cdef Val val
  for address, sigs in def_vals:
    for sg_name, def_val in sigs:
      val.name = sg_name
      val.address = address
      val.def_val = def_val
      val.sigs = sig_map[address]
      dbc.vals.push_back(val)

  dbc.num_msgs = dbc.msgs.size()
  dbc.num_vals = dbc.vals.size()
  dbc_register(dbc)

def ensure_dbc(dbc_name) :
  lock = threading.Lock()
  with(lock):
    if not dbc_lookup(dbc_name):
      checksum,msgs,vals= process(dbc_name)
      register_dbc(dbc_name, checksum, msgs, vals)

