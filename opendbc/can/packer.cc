#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <utility>

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
    MessageData &msg_data = msg_lookup[msg.address];
    msg_data.msg_size = msg.size;

    for (const auto& sig : msg.sigs) {
      msg_data.signals[sig.name] = &sig;

      if (sig.name == "COUNTER") {
        msg_data.counter_signal = &sig;
      } else if (sig.name == "CHECKSUM") {
        msg_data.checksum_signal = &sig;
      }
    }
  }
}

std::vector<uint8_t> CANPacker::pack(uint32_t address, const std::vector<SignalPackValue> &signals) {
  auto msg_it = msg_lookup.find(address);
  if (msg_it == msg_lookup.end()) {
    LOGE("undefined address %d", address);
    return {};
  }

  MessageData &msg = msg_it->second;
  std::vector<uint8_t> ret(msg.msg_size, 0);

  // set all values for all given signal/value pairs
  bool counter_set = false;
  for (const auto& sigval : signals) {
    auto sig_it = msg.signals.find(sigval.name);
    if (sig_it == msg.signals.end()) {
      // TODO: do something more here. invalid flag like CANParser?
      LOGE("undefined signal %s - %d\n", sigval.name.c_str(), address);
      continue;
    }

    const Signal *sig = sig_it->second;
    int64_t ival = (int64_t)(round((sigval.value - sig->offset) / sig->factor));
    if (ival < 0) {
      ival = (1ULL << sig->size) + ival;
    }
    set_value(ret, *sig, ival);

    if (sig == msg.counter_signal) {
      msg.counter_value = sigval.value;
      counter_set = true;
    }
  }

  // set message counter if not already set
  if (msg.counter_signal && !counter_set) {
    set_value(ret, *msg.counter_signal, msg.counter_value);
    msg.counter_value = (msg.counter_value + 1) % (1 << msg.counter_signal->size);
  }

  // set message checksum
  if (msg.checksum_signal && msg.checksum_signal->calc_checksum) {
    unsigned int checksum = msg.checksum_signal->calc_checksum(address, *msg.checksum_signal, ret);
    set_value(ret, *msg.checksum_signal, checksum);
  }

  return ret;
}

// This function has a definition in common.h and is used in PlotJuggler
const Msg* CANPacker::lookup_message(uint32_t address) {
  return dbc->addr_to_msg.at(address);
}
