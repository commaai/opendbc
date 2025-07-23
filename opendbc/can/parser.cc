#include <algorithm>
#include <cassert>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include "opendbc/can/common.h"


// ***** MessageState *****

int64_t get_raw_value(const std::vector<uint8_t> &msg, const Signal &sig) {
  int64_t ret = 0;

  int i = sig.msb / 8;
  int bits = sig.size;
  while (i >= 0 && i < msg.size() && bits > 0) {
    int lsb = (int)(sig.lsb / 8) == i ? sig.lsb : i*8;
    int msb = (int)(sig.msb / 8) == i ? sig.msb : (i+1)*8 - 1;
    int size = msb - lsb + 1;

    uint64_t d = (msg[i] >> (lsb - (i*8))) & ((1ULL << size) - 1);
    ret |= d << (bits - size);

    bits -= size;
    i = sig.is_little_endian ? i-1 : i+1;
  }
  return ret;
}


bool MessageState::parse(uint64_t nanos, const std::vector<uint8_t> &dat) {
  std::vector<double> tmp_vals(signals.size());
  bool checksum_failed = false;
  bool counter_failed = false;

  if (first_seen_nanos == 0) {
    first_seen_nanos = nanos;
  }

  for (int i = 0; i < signals.size(); i++) {
    const auto &sig = signals[i];

    int64_t tmp = get_raw_value(dat, sig);
    if (sig.is_signed) {
      tmp -= ((tmp >> (sig.size-1)) & 0x1) ? (1ULL << sig.size) : 0;
    }

    if (!ignore_checksum) {
      if (sig.calc_checksum != nullptr && sig.calc_checksum(address, sig, dat) != tmp) {
        checksum_failed = true;
      }
    }

    if (!ignore_counter) {
      if (sig.type == SignalType::COUNTER && !update_counter(tmp, sig.size)) {
        counter_failed = true;
      }
    }

    tmp_vals[i] = tmp * sig.factor + sig.offset;
  }

  // only update values + timestamps  if both checksum and counter are valid
  if (checksum_failed || counter_failed) {
    LOGE_100("0x%X message checks failed, checksum failed %d, counter failed %d", address, checksum_failed, counter_failed);
    return false;
  }

  for (int i = 0; i < signals.size(); i++) {
    vals[i] = tmp_vals[i];
    all_vals[i].push_back(vals[i]);
  }
  timestamps.push_back(nanos);

  const int max_buffer = 500; // allows 0.5s history for 1kHz (highest freq we've seen)
  while (timestamps.size() > max_buffer) {
    timestamps.pop_front();
  }

  // learn message frequency
  if (frequency < 1e-5) {
    double dt = (timestamps.back() - timestamps.front())*1e-9;
    if ((timestamps.size() >= 3 && (dt > 1.0f)) || (timestamps.size() >= max_buffer)) {
      frequency = std::min(timestamps.size() / dt, (double)100.0f);  // 100Hz max for checks
      timeout_threshold = (1000000000ULL / frequency) * 10;  // timeout on 10x expected freq
    }
  }

  return true;
}


bool MessageState::update_counter(int64_t cur_count, int cnt_size) {
  if (((counter + 1) & ((1 << cnt_size) - 1)) != cur_count) {
    counter_fail = std::min((int)counter_fail + 1, MAX_BAD_COUNTER);
    if (counter_fail > 1) {
      INFO("0x%X COUNTER FAIL #%d -- %d -> %d\n", address, counter_fail, counter, (int)cur_count);
    }
  } else if (counter_fail > 0) {
    counter_fail--;
  }
  counter = cur_count;
  return counter_fail < MAX_BAD_COUNTER;
}

bool MessageState::valid(uint64_t current_nanos, bool bus_timeout) const {
  /*
    bad counters and checksums don't get added to timestamps, so those
    cases get caught here too.
  */

  if (ignore_alive) {
    return true;
  }

  const bool print = !bus_timeout && ((current_nanos - first_seen_nanos) > 7e9);
  if (timestamps.empty()) {
    if (print) LOGE_100("0x%X '%s' NOT SEEN", address, name.c_str());
    return false;
  }
  if (timeout_threshold > 0 && ((current_nanos - timestamps.back()) > timeout_threshold)) {
    if (print) LOGE_100("0x%X '%s' TIMED OUT", address, name.c_str());
    return false;
  }
  return true;
}

// ***** CANParser *****

CANParser::CANParser(int abus, const std::string& dbc_name, const std::vector<std::pair<uint32_t, int>> &messages)
  : bus(abus) {
  dbc = dbc_lookup(dbc_name);
  assert(dbc);

  for (const auto& [address, frequency] : messages) {
    // disallow duplicate message checks
    if (message_states.find(address) != message_states.end()) {
      std::stringstream is;
      is << "Duplicate Message Check: " << address;
      throw std::runtime_error(is.str());
    }

    MessageState &state = message_states[address];
    state.address = address;
    // hack for signals whose frequencies vary more than 10x
    // TODO: figure out a good way to handle this without passing it in...
    if (frequency < 10 && frequency > 0) {
      state.frequency = frequency;
      state.timeout_threshold = (1000000000ULL / frequency) * 10;  // timeout on 10x expected freq
    }
    state.ignore_alive = (frequency == 0);

    const Msg *msg = dbc->addr_to_msg.at(address);
    state.name = msg->name;
    state.size = msg->size;
    assert(state.size <= 64);  // max signal size is 64 bytes

    // track all signals for this message
    state.signals = msg->sigs;
    state.vals.resize(msg->sigs.size());
    state.all_vals.resize(msg->sigs.size());
  }
}

CANParser::CANParser(int abus, const std::string& dbc_name, bool ignore_checksum, bool ignore_counter)
  : bus(abus) {
  // Add all messages and signals

  dbc = dbc_lookup(dbc_name);
  assert(dbc);

  for (const auto& msg : dbc->msgs) {
    MessageState state = {
      .name = msg.name,
      .address = msg.address,
      .size = msg.size,
      .ignore_checksum = ignore_checksum,
      .ignore_counter = ignore_counter,
    };

    for (const auto& sig : msg.sigs) {
      state.signals.push_back(sig);
      state.vals.push_back(0);
      state.all_vals.push_back({});
    }

    message_states[state.address] = state;
  }
}

std::set<uint32_t> CANParser::update(const std::vector<CanData> &can_data) {
  // Clear all_values
  for (auto &state : message_states) {
    for (auto &vals : state.second.all_vals) vals.clear();
  }

  std::set<uint32_t> updated_addresses;
  for (const auto &c : can_data) {
    UpdateCans(c, updated_addresses);
    UpdateValid(c.nanos);
  }

  return updated_addresses;
}

void CANParser::UpdateCans(const CanData &can, std::set<uint32_t> &updated_addresses) {
  bool bus_empty = true;

  for (const auto &frame : can.frames) {
    if (frame.src != bus) {
      continue;
    }
    bus_empty = false;

    auto state_it = message_states.find(frame.address);
    if (state_it == message_states.end()) {
      continue;
    }
    if (frame.dat.size() > 64) {
      DEBUG("got message longer than 64 bytes: 0x%X %zu\n", frame.address, frame.dat.size());
      continue;
    }

    // TODO: this actually triggers for some cars. fix and enable this
    //if (dat.size() != state_it->second.size) {
    //  DEBUG("got message with unexpected length: expected %d, got %zu for %d", state_it->second.size, dat.size(), cmsg.getAddress());
    //  continue;
    //}

    if (state_it->second.parse(can.nanos, frame.dat)) {
      updated_addresses.insert(state_it->first);
    }
  }

  // update bus timeout
  if (!bus_empty) {
    last_nonempty_nanos = can.nanos;
  }
  uint64_t bus_timeout_threshold = 500*1e6;
  for (const auto& kv : message_states) {
    const auto& state = kv.second;
    if (state.timeout_threshold > 0) {
      bus_timeout_threshold = std::min(bus_timeout_threshold, state.timeout_threshold);
    }
  }
  bus_timeout = (can.nanos - last_nonempty_nanos) > bus_timeout_threshold;
}

void CANParser::UpdateValid(uint64_t nanos) {
  bool valid = true;
  bool counters_valid = true;

  for (auto& kv : message_states) {
    const auto& state = kv.second;
    if (state.counter_fail >= MAX_BAD_COUNTER) {
      counters_valid = false;
    }
    if (!state.valid(nanos, bus_timeout)) {
      valid = false;
    }
  }

  can_invalid_cnt = valid ? 0 : std::min(can_invalid_cnt + 1, CAN_INVALID_CNT);
  can_valid = (can_invalid_cnt < CAN_INVALID_CNT) && counters_valid;
}

