#include <algorithm>
#include <cassert>
#include <cstring>
#include <limits>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "common.h"


int64_t get_raw_value(const std::vector<uint8_t> &msg, const Signal &sig) {
  int64_t ret = 0;

  int i = sig.msb / 8;
  int bits = sig.size;
  while (i >= 0 && i < msg.size() && bits > 0) {
    int lsb = (int)(sig.lsb / 8) == i ? sig.lsb : i*8;
    int msb = (int)(sig.msb / 8) == i ? sig.msb : (i+1)*8 - 1;
    int size = msb - lsb + 1;

    uint8_t d = (msg[i] >> (lsb - (i*8))) & ((1ULL << size) - 1);
    ret |= d << (bits - size);

    bits -= size;
    i = sig.is_little_endian ? i-1 : i+1;
  }
  return ret;
}


bool MessageState::parse(uint64_t sec, const std::vector<uint8_t> &dat) {

  for (int i = 0; i < parse_sigs.size(); i++) {
    auto &sig = parse_sigs[i];

    int64_t tmp = get_raw_value(dat, sig);
    if (sig.is_signed) {
      tmp -= ((tmp >> (sig.size-1)) & 0x1) ? (1ULL << sig.size) : 0;
    }

    //DEBUG("parse 0x%X %s -> %ld\n", address, sig.name, tmp);

    bool checksum_failed = false;
    if (!ignore_checksum) {
      if (sig.type == SignalType::HONDA_CHECKSUM && honda_checksum(address, dat) != tmp) {
        checksum_failed = true;
      } else if (sig.type == SignalType::TOYOTA_CHECKSUM && toyota_checksum(address, dat) != tmp) {
        checksum_failed = true;
      } else if (sig.type == SignalType::VOLKSWAGEN_CHECKSUM && volkswagen_crc(address, dat) != tmp) {
        checksum_failed = true;
      } else if (sig.type == SignalType::SUBARU_CHECKSUM && subaru_checksum(address, dat) != tmp) {
        checksum_failed = true;
      } else if (sig.type == SignalType::CHRYSLER_CHECKSUM && chrysler_checksum(address, dat) != tmp) {
        checksum_failed = true;
      } else if (sig.type == SignalType::PEDAL_CHECKSUM && pedal_checksum(dat) != tmp) {
        checksum_failed = true;
      }
    }

    bool counter_failed = false;
    if (!ignore_counter) {
      if (sig.type == SignalType::HONDA_COUNTER || sig.type == SignalType::VOLKSWAGEN_COUNTER || sig.type == SignalType::PEDAL_COUNTER) {
        counter_failed = !update_counter_generic(tmp, sig.size);
      }
    }

    if (checksum_failed || counter_failed) {
      WARN("0x%X message checks failed, checksum failed %d, counter failed %d\n", address, checksum_failed, counter_failed);
      return false;
    }

    // TODO: these may get updated if the invalid or checksum gets checked later
    vals[i] = tmp * sig.factor + sig.offset;
    all_vals[i].push_back(vals[i]);
  }
  seen = sec;

  return true;
}


bool MessageState::update_counter_generic(int64_t v, int cnt_size) {
  uint8_t old_counter = counter;
  counter = v;
  if (((old_counter+1) & ((1 << cnt_size) -1)) != v) {
    counter_fail += 1;
    if (counter_fail > 1) {
      INFO("0x%X COUNTER FAIL #%d -- %d -> %d\n", address, counter_fail, old_counter, (int)v);
    }
    if (counter_fail >= MAX_BAD_COUNTER) {
      return false;
    }
  } else if (counter_fail > 0) {
    counter_fail--;
  }
  return true;
}


CANParser::CANParser(int abus, const std::string& dbc_name,
          const std::vector<MessageParseOptions> &options,
          const std::vector<SignalParseOptions> &sigoptions)
  : bus(abus), aligned_buf(kj::heapArray<capnp::word>(1024)) {

  dbc = dbc_lookup(dbc_name);
  assert(dbc);
  init_crc_lookup_tables();

  bus_timeout_threshold = std::numeric_limits<uint64_t>::max();

  for (const auto& op : options) {
    MessageState &state = message_states[op.address];
    state.address = op.address;
    // state.check_frequency = op.check_frequency,

    // msg is not valid if a message isn't received for 10 consecutive steps
    if (op.check_frequency > 0) {
      state.check_threshold = (1000000000ULL / op.check_frequency) * 10;

      // bus timeout threshold should be 10x the fastest msg
      bus_timeout_threshold = std::min(bus_timeout_threshold, state.check_threshold);
    }

    const Msg* msg = NULL;
    for (int i = 0; i < dbc->num_msgs; i++) {
      if (dbc->msgs[i].address == op.address) {
        msg = &dbc->msgs[i];
        break;
      }
    }
    if (!msg) {
      fprintf(stderr, "CANParser: could not find message 0x%X in DBC %s\n", op.address, dbc_name.c_str());
      assert(false);
    }

    state.size = msg->size;
    assert(state.size < 64);  // max signal size is 64 bytes

    // track checksums and counters for this message
    for (int i = 0; i < msg->num_sigs; i++) {
      const Signal *sig = &msg->sigs[i];
      if (sig->type != SignalType::DEFAULT) {
        state.parse_sigs.push_back(*sig);
        state.vals.push_back(0);
        state.all_vals.push_back({});
      }
    }

    // track requested signals for this message
    for (const auto& sigop : sigoptions) {
      if (sigop.address != op.address) continue;

      for (int i = 0; i < msg->num_sigs; i++) {
        const Signal *sig = &msg->sigs[i];
        if (strcmp(sig->name, sigop.name) == 0
            && sig->type == SignalType::DEFAULT) {
          state.parse_sigs.push_back(*sig);
          state.vals.push_back(0);
          state.all_vals.push_back({});
          break;
        }
      }
    }
  }
}

CANParser::CANParser(int abus, const std::string& dbc_name, bool ignore_checksum, bool ignore_counter)
  : bus(abus) {
  // Add all messages and signals

  dbc = dbc_lookup(dbc_name);
  assert(dbc);
  init_crc_lookup_tables();

  for (int i = 0; i < dbc->num_msgs; i++) {
    const Msg* msg = &dbc->msgs[i];
    MessageState state = {
      .address = msg->address,
      .size = msg->size,
      .ignore_checksum = ignore_checksum,
      .ignore_counter = ignore_counter,
    };

    for (int j = 0; j < msg->num_sigs; j++) {
      const Signal *sig = &msg->sigs[j];
      state.parse_sigs.push_back(*sig);
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

  last_sec = event.getLogMonoTime();

  auto cans = sendcan ? event.getSendcan() : event.getCan();
  UpdateCans(last_sec, cans);

  UpdateValid(last_sec);
}

void CANParser::UpdateCans(uint64_t sec, const capnp::List<cereal::CanData>::Reader& cans) {
  //DEBUG("got %d messages\n", cans.size());

  bool bus_empty = true;

  // parse the messages
  for (int i = 0; i < cans.size(); i++) {
    auto cmsg = cans[i];
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
    state_it->second.parse(sec, data);
  }

  // update bus timeout
  if (!bus_empty) {
    last_nonempty_sec = sec;
  }
  bus_timeout = (sec - last_nonempty_sec) > bus_timeout_threshold;
}
#endif

void CANParser::UpdateCans(uint64_t sec, const capnp::DynamicStruct::Reader& cmsg) {
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
  state_it->second.parse(sec, data);
}

void CANParser::UpdateValid(uint64_t sec) {
  can_valid = true;
  for (const auto& kv : message_states) {
    const auto& state = kv.second;
    if (state.check_threshold > 0 && (sec - state.seen) > state.check_threshold) {
      if (state.seen > 0) {
        DEBUG("0x%X TIMEOUT\n", state.address);
      } else {
        DEBUG("0x%X MISSING\n", state.address);
      }
      can_valid = false;
    }
  }
}

std::vector<SignalValue> CANParser::query_latest() {
  std::vector<SignalValue> ret;

  for (auto& kv : message_states) {
    auto& state = kv.second;
    if (last_sec != 0 && state.seen != last_sec) continue;

    for (int i = 0; i < state.parse_sigs.size(); i++) {
      const Signal &sig = state.parse_sigs[i];
      ret.push_back((SignalValue){
        .address = state.address,
        .name = sig.name,
        .value = state.vals[i],
        .all_values = state.all_vals[i],
      });
      state.all_vals[i].clear();
    }
  }

  return ret;
}
