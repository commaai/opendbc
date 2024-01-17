# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

from libcpp.vector cimport vector
from .common cimport CANPacker as cpp_CANPacker
from .common cimport SignalPackValue

cdef class CANPacker:
  cdef cpp_CANPacker *packer

  def __init__(self, dbc_name):
    self.packer = new cpp_CANPacker(dbc_name)

  cpdef make_can_msg(self, name_or_addr, bus, values):
    cdef vector[SignalPackValue] values_thing
    values_thing.reserve(len(values))
    cdef SignalPackValue spv
    for name, value in values.iteritems():
      spv.name = name.encode("utf8")
      spv.value = value
      values_thing.push_back(spv)

    ret = self.packer.pack(name_or_addr, values_thing)
    return [ret.first, 0, (<char *>ret.second.data())[:ret.second.size()], bus]
