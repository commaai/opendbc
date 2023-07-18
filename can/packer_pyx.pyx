# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

from libc.stdint cimport uint8_t, uint32_t
from libcpp.vector cimport vector

from .common cimport CANPacker as cpp_CANPacker
from .common cimport dbc_lookup, SignalPackValue


cdef class CANPacker:
  cdef cpp_CANPacker *packer

  def __init__(self, dbc_name):
    if not dbc_lookup(dbc_name):
      raise RuntimeError(f"Can't lookup {dbc_name}")

    self.packer = new cpp_CANPacker(dbc_name)

  cdef vector[uint8_t] pack(self, addr, values):
    cdef vector[SignalPackValue] values_thing
    values_thing.reserve(len(values))
    cdef SignalPackValue spv

    for name, value in values.iteritems():
      spv.name = name.encode("utf8")
      spv.value = value
      values_thing.push_back(spv)

    return self.packer.pack(addr, values_thing)

  cpdef make_can_msg(self, name_or_addr, bus, values) except +RuntimeError:
    cdef uint32_t addr
    if type(name_or_addr) == int:
      addr = name_or_addr
    else:
      addr = self.packer.address_from_name(name_or_addr.encode("utf8"))

    cdef vector[uint8_t] val = self.pack(addr, values)
    return [addr, 0, (<char *>&val[0])[:val.size()], bus]
