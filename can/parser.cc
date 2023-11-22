#include <algorithm>
#include <cassert>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <sstream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "cereal/logger/logger.h"
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


bool MessageState::parse(uint64_t nanos, const std::vector<uint8_t> &dat) {
  std::vector<double> tmp_vals(parse_sigs.size());
  bool checksum_failed = false;
  bool counter_failed = false;

  for (int i = 0; i < parse_sigs.size(); i++) {
    const auto &sig = parse_sigs[i];

    int64_t tmp = get_raw_value(dat, sig);
    if (sig.is_signed) {
      tmp -= ((tmp >> (sig.size-1)) & 0x1) ? (1ULL << sig.size) : 0;
    }

    //DEBUG("parse 0x%X %s -> %ld\n", address, sig.name, tmp);

    if (!ignore_checksum) {
      if (sig.calc_checksum != nullptr && sig.calc_checksum(address, sig, dat) != tmp) {
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
    LOGE("0x%X message checks failed, checksum failed %d, counter failed %d", address, checksum_failed, counter_failed);
    return false;
  }

  for (int i = 0; i < parse_sigs.size(); i++) {
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
      INFO("0x%X COUNTER FAIL #%d -- %d -> %d\n", address, counter_fail, counter, (int)v);
    }
  } else if (counter_fail > 0) {
    counter_fail--;
  }
  counter = v;
  return counter_fail < MAX_BAD_COUNTER;
}


CANParser::CANParser(int abus, const std::string& dbc_name, const std::vector<std::pair<uint32_t, int>> &messages)
  : bus(abus), aligned_buf(kj::heapArray<capnp::word>(1024)) {
  dbc = dbc_lookup(dbc_name);
  assert(dbc);
  init_crc_lookup_tables();

  bus_timeout_threshold = std::numeric_limits<uint64_t>::max();

  for (const auto& [address, frequency] : messages) {
    // disallow duplicate message checks
    if (message_states.find(address) != message_states.end()) {
      std::stringstream is;
      is << "Duplicate Message Check: " << address;
      throw std::runtime_error(is.str());
    }

    MessageState &state = message_states[address];
    state.address = address;
    // state.check_frequency = op.check_frequency,

    // msg is not valid if a message isn't received for 10 consecutive steps
    if (frequency > 0) {
      state.check_threshold = (1000000000ULL / frequency) * 10;

      // bus timeout threshold should be 10x the fastest msg
      bus_timeout_threshold = std::min(bus_timeout_threshold, state.check_threshold);
    }

    const Msg* msg = NULL;
    for (const auto& m : dbc->msgs) {
      if (m.address == address) {
        msg = &m;
        break;
      }
    }
    if (!msg) {
      fprintf(stderr, "CANParser: could not find message 0x%X in DBC %s\n", address, dbc_name.c_str());
      assert(false);
    }

    state.name = msg->name;
    state.size = msg->size;
    assert(state.size <= 64);  // max signal size is 64 bytes

    // track all signals for this message
    state.parse_sigs = msg->sigs;
    state.vals.resize(msg->sigs.size());
    state.all_vals.resize(msg->sigs.size());
  }
}

CANParser::CANParser(int abus, const std::string& dbc_name, bool ignore_checksum, bool ignore_counter)
  : bus(abus) {
  // Add all messages and signals

  dbc = dbc_lookup(dbc_name);
  assert(dbc);
  init_crc_lookup_tables();

  for (const auto& msg : dbc->msgs) {
    MessageState state = {
      .name = msg.name,
      .address = msg.address,
      .size = msg.size,
      .ignore_checksum = ignore_checksum,
      .ignore_counter = ignore_counter,
    };

    for (const auto& sig : msg.sigs) {
      state.parse_sigs.push_back(sig);
      state.vals.push_back(0);
      state.all_vals.push_back({});
    }

    message_states[state.address] = state;
  }
}

#ifndef DYNAMIC_CAPNP
void CANParser::update_string(const std::string &data, bool sendcan) {
  // format for board, make copy due to alignment issues.
  const size_t buf_size = (data.length() / sizeof(capnp::word)) + 1;
  if (aligned_buf.size() < buf_size) {
    aligned_buf = kj::heapArray<capnp::word>(buf_size);
  }
  memcpy(aligned_buf.begin(), data.data(), data.length());

  // extract the messages
  capnp::FlatArrayMessageReader cmsg(aligned_buf.slice(0, buf_size));
  cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();

  if (first_nanos == 0) {
    first_nanos = event.getLogMonoTime();
  }
  last_nanos = event.getLogMonoTime();

  auto cans = sendcan ? event.getSendcan() : event.getCan();
  UpdateCans(last_nanos, cans);

  UpdateValid(last_nanos);
}

void CANParser::update_strings(const std::vector<std::string> &data, std::vector<SignalValue> &vals, bool sendcan) {
  uint64_t current_nanos = 0;
  for (const auto &d : data) {
    update_string(d, sendcan);
    if (current_nanos == 0) {
      current_nanos = last_nanos;
    }
  }
  query_latest(vals, current_nanos);
}

void CANParser::UpdateCans(uint64_t nanos, const capnp::List<cereal::CanData>::Reader& cans) {
  //DEBUG("got %d messages\n", cans.size());

  bool bus_empty = true;

  // parse the messages
  for (const auto cmsg : cans) {
    if (cmsg.getSrc() != bus) {
      // DEBUG("skip %d: wrong bus\n", cmsg.getAddress());
      continue;
    }
    bus_empty = false;

    auto state_it = message_states.find(cmsg.getAddress());
    if (state_it == message_states.end()) {
      // DEBUG("skip %d: not specified\n", cmsg.getAddress());
      continue;
    }

    auto dat = cmsg.getDat();

    if (dat.size() > 64) {
      DEBUG("got message longer than 64 bytes: 0x%X %zu\n", cmsg.getAddress(), dat.size());
      continue;
    }

    // TODO: this actually triggers for some cars. fix and enable this
    //if (dat.size() != state_it->second.size) {
    //  DEBUG("got message with unexpected length: expected %d, got %zu for %d", state_it->second.size, dat.size(), cmsg.getAddress());
    //  continue;
    //}

    std::vector<uint8_t> data(dat.size(), 0);
    memcpy(data.data(), dat.begin(), dat.size());
    state_it->second.parse(nanos, data);
  }

  // update bus timeout
  if (!bus_empty) {
    last_nonempty_nanos = nanos;
  }
  bus_timeout = (nanos - last_nonempty_nanos) > bus_timeout_threshold;
}
#endif

void CANParser::UpdateCans(uint64_t nanos, const capnp::DynamicStruct::Reader& cmsg) {
  // assume message struct is `cereal::CanData` and parse
  assert(cmsg.has("address") && cmsg.has("src") && cmsg.has("dat") && cmsg.has("busTime"));

  if (cmsg.get("src").as<uint8_t>() != bus) {
    DEBUG("skip %d: wrong bus\n", cmsg.get("address").as<uint32_t>());
    return;
  }

  auto state_it = message_states.find(cmsg.get("address").as<uint32_t>());
  if (state_it == message_states.end()) {
    DEBUG("skip %d: not specified\n", cmsg.get("address").as<uint32_t>());
    return;
  }

  auto dat = cmsg.get("dat").as<capnp::Data>();
  if (dat.size() > 64) return; // shouldn't ever happen
  std::vector<uint8_t> data(dat.size(), 0);
  memcpy(data.data(), dat.begin(), dat.size());
  state_it->second.parse(nanos, data);
}

void CANParser::UpdateValid(uint64_t nanos) {
  const bool show_missing = (last_nanos - first_nanos) > 8e9;

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
          LOGE("0x%X '%s' NOT SEEN", state.address, state.name.c_str());
        } else if (timed_out) {
          LOGE("0x%X '%s' TIMED OUT", state.address, state.name.c_str());
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

    for (int i = 0; i < state.parse_sigs.size(); i++) {
      const Signal &sig = state.parse_sigs[i];
      SignalValue &v = vals.emplace_back();
      v.address = state.address;
      v.ts_nanos = state.last_seen_nanos;
      v.name = sig.name;
      v.value = state.vals[i];
      v.all_values = state.all_vals[i];
      state.all_vals[i].clear();
    }
  }
}
