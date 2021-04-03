# distutils: language = c++
#cython: language_level=3

from libc.stdint cimport uint32_t, uint64_t, uint16_t
from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.string cimport string
from libcpp.unordered_set cimport unordered_set
from libcpp cimport bool


cdef extern from "common_dbc.h":
  ctypedef enum SignalType:
    DEFAULT,
    HONDA_CHECKSUM,
    HONDA_COUNTER,
    TOYOTA_CHECKSUM,
    PEDAL_CHECKSUM,
    PEDAL_COUNTER,
    VOLKSWAGEN_CHECKSUM,
    VOLKSWAGEN_COUNTER,
    SUBARU_CHECKSUM,
    CHRYSLER_CHECKSUM

  cdef struct Signal:
    string name
    int b1, b2, bo
    bool is_signed
    double factor, offset
    bool is_little_endian
    SignalType type

  cdef struct Msg:
    string name
    uint32_t address
    unsigned int size
    size_t num_sigs
    vector[Signal] sigs

  cdef struct Val:
    string name
    uint32_t address
    string def_val
    vector[Signal] sigs

  cdef struct DBC:
    string name
    size_t num_msgs
    vector[Msg] msgs
    vector[Val] vals
    size_t num_vals

  cdef struct SignalParseOptions:
    uint32_t address
    string name
    double default_value


  cdef struct MessageParseOptions:
    uint32_t address
    int check_frequency

  cdef struct SignalValue:
    uint32_t address
    uint16_t ts
    string name
    double value

  cdef struct SignalPackValue:
    string name
    double value


cdef extern from "common.h":
  cdef const DBC* dbc_lookup(const string);
  cdef void dbc_register(const DBC&)

  cdef cppclass CANParser:
    bool can_valid
    CANParser(int, string, vector[MessageParseOptions], vector[SignalParseOptions])
    void update_string(string, bool)
    vector[SignalValue] query_latest()

  cdef cppclass CANPacker:
   CANPacker(string)
   uint64_t pack(uint32_t, vector[SignalPackValue], int counter)
