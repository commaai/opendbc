# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

from libc.stdint cimport uint8_t
from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.string cimport string

from .common cimport CANPacker as cpp_CANPacker


cdef class CANPacker:
  cdef cpp_CANPacker *packer

  def __init__(self, dbc_name):
    self.packer = new cpp_CANPacker(dbc_name)

  cpdef make_can_msg(self, name_or_addr, bus, values) except +RuntimeError:
    cdef int addr = name_or_addr if type(name_or_addr) == int else self.packer.addressFromName(name_or_addr)
    cdef vector[uint8_t] val = self.packer.pack(addr, values)
    return [addr, 0, (<char *>&val[0])[:val.size()], bus]
