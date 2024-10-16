#include <algorithm>
#include <cassert>
#include <cstring>
#include <limits>
#include <stdexcept>


#include "opendbc/can/common.h"

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

MessageState::MessageState(const Msg *msg_ptr, uint64_t threshold, bool no_checksum, bool no_counter)
    : msg(*msg_ptr), check_threshold(threshold), ignore_checksum(no_checksum), ignore_counter(no_counter) {
  assert(msg.size <= 64 && "The maximum message size is 64 bytes");
  vals.resize(msg.sigs.size());
  all_vals.resize(msg.sigs.size());
}

bool MessageState::parse(uint64_t nanos, const std::vector<uint8_t> &dat) {
  std::vector<double> tmp_vals(msg.sigs.size());
  bool checksum_failed = false;
  bool counter_failed = false;

  for (int i = 0; i < msg.sigs.size(); ++i) {
    const auto &sig = msg.sigs[i];

    int64_t tmp = get_raw_value(dat, sig);
    if (sig.is_signed) {
      tmp -= ((tmp >> (sig.size-1)) & 0x1) ? (1ULL << sig.size) : 0;
    }

    //DEBUG("parse 0x%X %s -> %ld\n", address, sig.name, tmp);

    if (!ignore_checksum) {
      if (sig.calc_checksum != nullptr && sig.calc_checksum(msg.address, sig, dat) != tmp) {
        checksum_failed = true;
      }
    }

    if (!ignore_counter) {
      if (sig.type == SignalType::COUNTER && !update_counter_generic(tmp, sig.size)) {
        counter_failed = true;
      }
    }

    tmp_vals[i] = tmp * sig.factor + sig.offset;
  }

  // only update values if both checksum and counter are valid
  if (checksum_failed || counter_failed) {
    LOGE_100("0x%X message checks failed, checksum failed %d, counter failed %d", msg.address, checksum_failed, counter_failed);
    return false;
  }

  for (int i = 0; i < msg.sigs.size(); ++i) {
    vals[i] = tmp_vals[i];
    all_vals[i].push_back(vals[i]);
  }
  last_seen_nanos = nanos;

  return true;
}


bool MessageState::update_counter_generic(int64_t v, int cnt_size) {
  if (((counter + 1) & ((1 << cnt_size) -1)) != v) {
    counter_fail = std::min(counter_fail + 1, MAX_BAD_COUNTER);
    if (counter_fail > 1) {
      INFO("0x%X COUNTER FAIL #%d -- %d -> %d\n", msg.address, counter_fail, counter, (int)v);
    }
  } else if (counter_fail > 0) {
    counter_fail--;
  }
  counter = v;
  return counter_fail < MAX_BAD_COUNTER;
}


CANParser::CANParser(int abus, const std::string& dbc_name, const std::vector<std::pair<uint32_t, int>> &messages)
  : bus(abus) {
  dbc = dbc_lookup(dbc_name);
  assert(dbc);

  bus_timeout_threshold = std::numeric_limits<uint64_t>::max();

  for (const auto& [address, frequency] : messages) {
    // disallow duplicate message checks
    if (message_states.find(address) != message_states.end()) {
      throw std::runtime_error("Duplicate Message Check: " + std::to_string(address));
    }

    // msg is not valid if a message isn't received for 10 consecutive steps
    uint64_t check_threshold = frequency > 0 ? (1000000000ULL / frequency) * 10 : 0;
    if (check_threshold > 0) {
      // bus timeout threshold should be 10x the fastest msg
      bus_timeout_threshold = std::min(bus_timeout_threshold, check_threshold);
    }

    const Msg *msg = dbc->addr_to_msg.at(address);
    message_states.emplace(address, MessageState{msg, check_threshold});
  }
}

CANParser::CANParser(int abus, const std::string& dbc_name, bool ignore_checksum, bool ignore_counter)
  : bus(abus) {
  dbc = dbc_lookup(dbc_name);
  assert(dbc);
  for (const auto& msg : dbc->msgs) {
    message_states.emplace(msg.address, MessageState{&msg, 0, ignore_checksum, ignore_counter});
  }
}

void CANParser::update(const std::vector<CanData> &can_data, std::vector<SignalValue> &vals) {
  uint64_t current_nanos = 0;
  for (const auto &c : can_data) {
    if (first_nanos == 0) {
      first_nanos = c.nanos;
    }
    if (current_nanos == 0) {
      current_nanos = c.nanos;
    }
    last_nanos = c.nanos;

    UpdateCans(c);
    UpdateValid(last_nanos);
  }
  query_latest(vals, current_nanos);
}

void CANParser::UpdateCans(const CanData &can) {
  //DEBUG("got %zu messages\n", can.frames.size());

  bool bus_empty = true;

  for (const auto &frame : can.frames) {
    if (frame.src != bus) {
      // DEBUG("skip %d: wrong bus\n", cmsg.getAddress());
      continue;
    }
    bus_empty = false;

    auto state_it = message_states.find(frame.address);
    if (state_it == message_states.end()) {
      // DEBUG("skip %d: not specified\n", cmsg.getAddress());
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

    state_it->second.parse(can.nanos, frame.dat);
  }

  // update bus timeout
  if (!bus_empty) {
    last_nonempty_nanos = can.nanos;
  }
  bus_timeout = (can.nanos - last_nonempty_nanos) > bus_timeout_threshold;
}

void CANParser::UpdateValid(uint64_t nanos) {
  const bool show_missing = (nanos - first_nanos) > 8e9;

  bool _valid = true;
  bool _counters_valid = true;
  for (const auto& kv : message_states) {
    const auto& state = kv.second;

    if (state.counter_fail >= MAX_BAD_COUNTER) {
      _counters_valid = false;
    }

    const bool missing = state.last_seen_nanos == 0;
    const bool timed_out = (nanos - state.last_seen_nanos) > state.check_threshold;
    if (state.check_threshold > 0 && (missing || timed_out)) {
      if (show_missing && !bus_timeout) {
        if (missing) {
          LOGE_100("0x%X '%s' NOT SEEN", state.msg.address, state.msg.name.c_str());
        } else if (timed_out) {
          LOGE_100("0x%X '%s' TIMED OUT", state.msg.address, state.msg.name.c_str());
        }
      }
      _valid = false;
    }
  }
  can_invalid_cnt = _valid ? 0 : (can_invalid_cnt + 1);
  can_valid = (can_invalid_cnt < CAN_INVALID_CNT) && _counters_valid;
}

void CANParser::query_latest(std::vector<SignalValue> &vals, uint64_t last_ts) {
  if (last_ts == 0) {
    last_ts = last_nanos;
  }
  for (auto& kv : message_states) {
    auto& state = kv.second;
    if (last_ts != 0 && state.last_seen_nanos < last_ts) {
      continue;
    }

    for (int i = 0; i < state.msg.sigs.size(); ++i) {
      const Signal &sig = state.msg.sigs[i];
      SignalValue &v = vals.emplace_back();
      v.address = state.msg.address;
      v.ts_nanos = state.last_seen_nanos;
      v.name = sig.name;
      v.value = state.vals[i];
      v.all_values = state.all_vals[i];
      state.all_vals[i].clear();
    }
  }
}
