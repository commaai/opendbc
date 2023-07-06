#include <cassert>
#include <utility>
#include <algorithm>
#include <map>
#include <cmath>

#include "opendbc/can/common.h"


void set_value(std::vector<uint8_t> &msg, const Signal &sig, int64_t ival) {
  int i = sig.lsb / 8;
  int bits = sig.size;
  if (sig.size < 64) {
    ival &= ((1ULL << sig.size) - 1);
  }

  while (i >= 0 && i < msg.size() && bits > 0) {
    int shift = (int)(sig.lsb / 8) == i ? sig.lsb % 8 : 0;
    int size = std::min(bits, 8 - shift);

    msg[i] &= ~(((1ULL << size) - 1) << shift);
    msg[i] |= (ival & ((1ULL << size) - 1)) << shift;

    bits -= size;
    ival >>= size;
    i = sig.is_little_endian ? i+1 : i-1;
  }
}

CANPacker::CANPacker(const std::string& dbc_name) {
  dbc = dbc_lookup(dbc_name);
  assert(dbc);

  for (const auto& msg : dbc->msgs) {
    message_lookup[msg.address] = msg;
    for (const auto& sig : msg.sigs) {
      signal_lookup[std::make_pair(msg.address, std::string(sig.name))] = sig;
    }
  }
  init_crc_lookup_tables();
}

std::vector<uint8_t> CANPacker::pack(uint32_t address, const std::vector<SignalPackValue> &values) {
  // check parameters
  auto msg_it = message_lookup.find(address);
  if (msg_it == message_lookup.end()) {
    throw std::runtime_error("CanPacker::pack(): invalid address " + std::to_string(address));
  }

  for (const auto &v : values) {
    if (signal_lookup.find(std::make_pair(address, v.name)) == signal_lookup.end()) {
      throw std::runtime_error("CanPacker::pack(): undefined signal:" + v.name + " in " + std::to_string(address));
    }
  }

  // set all values for all given signal/value pairs
  bool counter_set = false;
  const auto &msg = msg_it->second;
  std::vector<uint8_t> ret(msg.size, 0);

  for (const auto &sig : msg.sigs) {
    auto value_it = std::find_if(values.begin(), values.end(), [name = sig.name](auto &v) { return v.name == name; });
    double v = (value_it != values.end()) ? value_it->value : 0.0;
    int64_t ival = (int64_t)(round((v - sig.offset) / sig.factor));
    if (ival < 0) {
      ival = (1ULL << sig.size) + ival;
    }
    if (ival != 0) {
      set_value(ret, sig, ival);
    }

    if (sig.name == "COUNTER" && value_it != values.end()) {
      counter_set = true;
      counters[address] = v;
    }
  }

  // set message counter
  auto sig_it_counter = signal_lookup.find(std::make_pair(address, "COUNTER"));
  if (!counter_set && sig_it_counter != signal_lookup.end()) {
    const auto &sig = sig_it_counter->second;
    auto &cnt = counters[address];
    set_value(ret, sig, cnt);
    cnt = (cnt + 1) % (1 << sig.size);
  }

  // set message checksum
  auto sig_it_checksum = signal_lookup.find(std::make_pair(address, "CHECKSUM"));
  if (sig_it_checksum != signal_lookup.end()) {
    const auto &sig = sig_it_checksum->second;
    if (sig.calc_checksum != nullptr) {
      unsigned int checksum = sig.calc_checksum(address, sig, ret);
      set_value(ret, sig, checksum);
    }
  }

  return ret;
}

// This function has a definition in common.h and is used in PlotJuggler
Msg* CANPacker::lookup_message(uint32_t address) {
  return &message_lookup[address];
}
