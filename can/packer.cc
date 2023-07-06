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

// set all values for all given signal/value pairs
std::vector<uint8_t> CANPacker::pack(uint32_t address, const std::vector<SignalPackValue> &signals) {
  auto msg_it = message_lookup.find(address);
  if (msg_it == message_lookup.end()) {
    throw std::runtime_error("CanPacker::pack(): invalid address " + std::to_string(address));
  }

  for (const auto &sig: signals) {
    if (signal_lookup.find({address, sig.name}) == signal_lookup.end()) {
      throw std::runtime_error("CanPacker::pack(): undefined signal:" + sig.name + " in " + std::to_string(address));
    }
  }

  std::vector<uint8_t> ret(message_lookup[address].size, 0);
  bool counter_set = false;
  for (const auto &sig : msg_it->second.sigs) {
    double value = 0.0;
    auto it = std::find_if(signals.begin(), signals.end(), [name = sig.name](auto &v) { return v.name == name; });
    if (it != signals.end()) {
      value = it->value;
    }
    int64_t ival = (int64_t)(round((value - sig.offset) / sig.factor));
    if (ival < 0) {
      ival = (1ULL << sig.size) + ival;
    }
    set_value(ret, sig, ival);

    if (sig.name == "COUNTER" && it != signals.end()) {
      counter_set = true;
      counters[address] = value;
    }
  }

  // set message counter
  auto sig_it_counter = signal_lookup.find(std::make_pair(address, "COUNTER"));
  if (!counter_set && sig_it_counter != signal_lookup.end()) {
    const auto& sig = sig_it_counter->second;

    if (counters.find(address) == counters.end()) {
      counters[address] = 0;
    }
    set_value(ret, sig, counters[address]);
    counters[address] = (counters[address] + 1) % (1 << sig.size);
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
