# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

from libc.stdint cimport uint8_t
from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.string cimport string

from .common cimport CANPacker as cpp_CANPacker
from .common cimport dbc_lookup, DBC


cdef class CANPacker:
  cdef cpp_CANPacker *packer

  def __init__(self, dbc_name):
    if not dbc_lookup(dbc_name):
      raise RuntimeError(f"Can't lookup {dbc_name}")
    self.packer = new cpp_CANPacker(dbc_name)

  cpdef make_can_msg(self, name_or_addr, bus, values) except +RuntimeError:
    cdef int addr
    if type(name_or_addr) == int:
      addr = name_or_addr
    else:
      addr = self.packer.addressFromName(name_or_addr)

    cdef vector[uint8_t] val = self.packer.pack(addr, values)
    return [addr, 0, (<char *>&val[0])[:val.size()], bus]
