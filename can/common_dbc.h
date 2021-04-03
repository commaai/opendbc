#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct SignalPackValue {
  std::string name;
  double value;
};

struct SignalParseOptions {
  uint32_t address;
  std::string name;
  double default_value;
};

struct MessageParseOptions {
  uint32_t address;
  int check_frequency;
};

struct SignalValue {
  uint32_t address;
  uint16_t ts;
  std::string name;
  double value;
};

enum SignalType {
  DEFAULT,
  HONDA_CHECKSUM,
  HONDA_COUNTER,
  TOYOTA_CHECKSUM,
  PEDAL_CHECKSUM,
  PEDAL_COUNTER,
  VOLKSWAGEN_CHECKSUM,
  VOLKSWAGEN_COUNTER,
  SUBARU_CHECKSUM,
  CHRYSLER_CHECKSUM,
};

struct Signal {
  std::string name;
  int b1, b2, bo;
  bool is_signed;
  double factor, offset;
  bool is_little_endian;
  SignalType type;
};

struct Msg {
  std::string name;
  uint32_t address;
  unsigned int size;
  size_t num_sigs;
  std::vector<Signal> sigs;
};

struct Val {
  std::string name;
  uint32_t address;
  std::string def_val;
  std::vector<Signal> sigs;
};

struct DBC {
  std::string name;
  size_t num_msgs;
  std::vector<Msg> msgs;
  std::vector<Val> vals;
  size_t num_vals;
};

const DBC* dbc_lookup(const std::string& dbc_name);
void dbc_register(const DBC& dbc);
