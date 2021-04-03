# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp cimport bool
from libc.stdint cimport uint32_t, uint64_t, uint16_t

import threading
from opendbc.can.process_dbc import process
from .common cimport dbc_lookup, SignalValue, DBC, Msg, SignalType, Signal, Val, dbc_register

cdef get_sigs(address, checksum_type, sigs):
  cdef vector[Signal] ret
  cdef Signal s
  for sig in sigs:
    b1 = sig.start_bit if sig.is_little_endian else (sig.start_bit//8)*8  + (-sig.start_bit-1) % 8
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
  cdef DBC dbc
  cdef Msg m
  cdef Signal sig
  cdef Val v
  sig_map = {}

  dbc.name = name
  for address, msg_name, msg_size, sigs in msgs:
    m.name = msg_name
    m.address = address
    m.size = msg_size
    m.sigs = get_sigs(address, checksum_type, sigs)
    m.num_sigs = m.sigs.size()
    dbc.msgs.push_back(m);

    sig_map[address] = m.sigs

  dbc.num_msgs = dbc.msgs.size()

  for address, sigs in def_vals:
    for sg_name, def_val in sigs:
      v.name = sg_name
      v.address = address
      v.def_val = def_val
      v.sigs = sig_map[address]
      dbc.vals.push_back(v)

  dbc.num_vals = dbc.vals.size()
  dbc_register(dbc)

def ensure_dbc(dbc_name) :
  lock = threading.Lock()
  with(lock):
    if not dbc_lookup(dbc_name):
      checksum,msgs,vals= process(dbc_name)
      register_dbc(dbc_name, checksum, msgs, vals)

