#include <assert.h>
#include <byteswap.h>
#include <limits.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <vector>

#include "common.h"
#include "common_dbc.h"

namespace {

#define DBC_ASSERT(condition, message)                                        \
  do {                                                                        \
    if (!(condition)) {                                                       \
      std::cerr << "Assertion `" #condition "` failed in " << __FILE__        \
                << " line " << __LINE__ << ": [" << dbc_name << "] " << message << std::endl; \
      std::terminate();                                                       \
    }                                                                         \
  } while (false)

template <typename... Args>
inline std::string str(const std::string& format, Args... args) {
  size_t size = snprintf(nullptr, 0, format.c_str(), args...) + 1;
  std::unique_ptr<char[]> buf(new char[size]);
  snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(), buf.get() + size - 1);
}

inline bool startswith(const std::string& str, const char* s) {
  return strncmp(str.c_str(), s, strlen(s)) == 0;
}

inline bool startswith(const std::string& str, std::initializer_list<const char*> s) {
  return s.end() != std::find_if(s.begin(), s.end(), [&](auto& it) { return strncmp(str.c_str(), it, strlen(it)) == 0; });
}

inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v") {
  s.erase(s.find_last_not_of(t) + 1);
  return s.erase(0, s.find_first_not_of(t));
}

typedef struct CheckSum {
  int checksum_size;
  int counter_size;
  int checksum_start_bit;
  int counter_start_bit;
  bool little_endian;
  SignalType checksum_type;
  SignalType counter_type;
} CheckSum;

enum ChecksumType {
  NONE,
  HONDA,
  TOYOTA,
  VOLKSWAGEN,
  SUBARU,
  CHRYSLER
};

CheckSum checksums[] = {
  {
    .checksum_size = 4,
    .counter_size = 2,
    .checksum_start_bit = 3,
    .counter_start_bit = 5,
    .little_endian = false,
    .checksum_type = HONDA_CHECKSUM,
    .counter_type = HONDA_COUNTER,
  },
  {
    .checksum_size = 8,
    .counter_size = -1,
    .checksum_start_bit = 7,
    .counter_start_bit = -1,
    .little_endian = false,
    .checksum_type = TOYOTA_CHECKSUM,
  },
  {
    .checksum_size = 8,
    .counter_size = 4,
    .checksum_start_bit = 0,
    .counter_start_bit = 0,
    .little_endian = true,
    .checksum_type = VOLKSWAGEN_CHECKSUM,
    .counter_type = VOLKSWAGEN_COUNTER,
  },
  {
    .checksum_size = 8,
    .counter_size = -1,
    .checksum_start_bit = 0,
    .counter_start_bit = -1,
    .little_endian = true,
    .checksum_type = SUBARU_CHECKSUM,
  },
  {
    .checksum_size = 8,
    .counter_size = -1,
    .checksum_start_bit = 7,
    .counter_start_bit = -1,
    .little_endian = false,
    .checksum_type = CHRYSLER_CHECKSUM,
  },
};

void set_signal_type(Signal& s, uint32_t address, CheckSum* chk, const std::string& dbc_name) {
  if (chk) {
    if (s.name == "CHECKSUM") {
      DBC_ASSERT(s.b2 == chk->checksum_size, str("CHECKSUM is not %d bits long", chk->checksum_size));
      DBC_ASSERT((s.b1 % 8) == chk->checksum_start_bit, " CHECKSUM starts at wrong bit");
      DBC_ASSERT(s.is_little_endian == chk->little_endian, "CHECKSUM has wrong endianness");
      s.type = chk->checksum_type;
    } else if (s.name == "COUNTER") {
      DBC_ASSERT(chk->counter_size == -1 || s.b2 == chk->counter_size, str("COUNTER is not %d bits long", chk->counter_size));
      DBC_ASSERT(chk->counter_start_bit == -1 || s.b1 % 8 == chk->counter_start_bit, "COUNTER starts at wrong bit");
      DBC_ASSERT(chk->little_endian == s.is_little_endian, "COUNTER has wrong endianness");
      s.type = chk->counter_type;
    }
  }
  if (address == 0x200 || address == 0x201) {
    if (s.name == "CHECKSUM_PEDAL") {
      DBC_ASSERT(s.b2 == 8, "PEDAL CHECKSUM is not 8 bits long");
      s.type = PEDAL_CHECKSUM;
    } else if (s.name == "COUNTER_PEDAL") {
      DBC_ASSERT(s.b2 == 4, "PEDAL COUNTER is not 4 bits long");
      s.type = PEDAL_COUNTER;
    }
  }
}

DBC* dbc_parse(const std::string& dbc_name) {
  std::regex bo_regexp(R"(^BO\_ (\w+) (\w+) *: (\w+) (\w+))");
  std::regex sg_regexp(R"(^SG\_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))");
  std::regex sgm_regexp(R"(^SG\_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))");
  std::regex val_regexp(R"(VAL\_ (\w+) (\w+) (\s*[-+]?[0-9]+\s+\".+?\"[^;]*))");

  CheckSum *checksum_ = nullptr;
  if (startswith(dbc_name, {"honda_", "acura_"})) {
    checksum_ = &checksums[HONDA];
  } else if (startswith(dbc_name, {"toyota_", "lexus_"})) {
    checksum_ = &checksums[TOYOTA];
  } else if (startswith(dbc_name, {"vw_", "volkswagen_", "audi_", "seat_", "skoda_"})) {
    checksum_ = &checksums[VOLKSWAGEN];
  } else if (startswith(dbc_name, "subaru_global_")) {
    checksum_ = &checksums[SUBARU];
  } else if (startswith(dbc_name, "chrysler_", "stellantis_")) {
    checksum_ = &checksums[CHRYSLER];
  }

  uint32_t address = 0;
  std::set<uint32_t> address_set;
  std::set<std::string> msg_name_set;
  std::map<uint32_t, std::vector<Signal>> signals;
  DBC* dbc = new DBC;
  dbc->name = dbc_name;

  std::ifstream infile("../" + dbc_name + ".dbc");
  DBC_ASSERT(infile, "failed open dbc file");
  std::string line;
  while (std::getline(infile, line)) {
    line = trim(line);
    std::smatch match;
    if (startswith(line, "BO_ ")) {
      // new group
      bool ret = std::regex_match(line, match, bo_regexp);
      DBC_ASSERT(ret, str("bad BO %s", line.c_str()));

      address = std::stoi(match[1].str());  // could be hex
      std::string name = match[2].str();
      uint32_t size = std::stoi(match[3].str());

      DBC_ASSERT(address_set.find(address) == address_set.end(),
                 str("Duplicate address detected : %d", address));
      address_set.insert(address);

      DBC_ASSERT(msg_name_set.find(name) == msg_name_set.end(),
                 str("Duplicate message name : %s", name.c_str()));
      msg_name_set.insert(name);

      // Msg m = {.name = name, .size = size, .address = address};
      Msg& msg = dbc->msgs.emplace_back();
      msg.address = address;
      msg.name = name;
      msg.size = size;
    } else if (startswith(line, "SG_ ")) {
      // new signal
      int offset = 0;
      if (!std::regex_search(line, match, sg_regexp)) {
        bool ret = std::regex_search(line, match, sgm_regexp);
        DBC_ASSERT(ret, str("bad SG %s", line.c_str()));
        offset = 1;
      }
      Signal& sig = signals[address].emplace_back();
      sig.name = match[1].str();
      sig.b1 = std::stoi(match[offset + 2].str());
      sig.b2 = std::stoi(match[offset + 3].str());
      sig.is_little_endian = std::stoi(match[offset + 4].str()) == 1;
      sig.is_signed = match[offset + 5].str() == "-";
      sig.factor = std::stof(match[offset + 6].str());
      sig.offset = std::stof(match[offset + 7].str());
      set_signal_type(sig, address, checksum_, dbc_name);
      if (!sig.is_little_endian) {
        uint64_t b1 = sig.b1;
        sig.b1 = std::floor(b1 / 8) * 8 + (-b1 - 1) % 8;
      }
      sig.bo = 64 - (sig.b1 + sig.b2);
    } else if (startswith(line, "VAL_ ")) {
      // new signal value/definition
      bool ret = std::regex_search(line, match, val_regexp);
      DBC_ASSERT(ret, str("bad VAL %s", line.c_str()));

      auto& val = dbc->vals.emplace_back();
      val.address = std::stoi(match[1].str());  // could be hex
      val.name = match[2].str();

      auto defvals = match[3].str();
      std::regex regex{R"([\"]+)"};  // split on "
      std::sregex_token_iterator it{defvals.begin(), defvals.end(), regex, -1};
      // convert strings to UPPER_CASE_WITH_UNDERSCORES
      std::vector<std::string> words{it, {}};
      for (auto& w : words) {
        w = trim(w);
        std::transform(w.begin(), w.end(), w.begin(), ::toupper);
        std::replace(w.begin(), w.end(), ' ', '_');
      }

      // join string
      std::stringstream s;
      std::copy(words.begin(), words.end(), std::ostream_iterator<std::string>(s, " "));
      val.def_val = s.str();
    }
  }

  for (auto& m : dbc->msgs) {
    m.sigs = signals.at(m.address);
  }
  for (auto& v : dbc->vals) {
    v.sigs = signals.at(v.address);
  }
  return dbc;
}

}  // namespace

const DBC* dbc_lookup(const std::string& dbc_name) {
  static std::mutex lock;
  static std::map<std::string, DBC*> dbcs;

  std::unique_lock lk(lock);

  auto it = dbcs.find(dbc_name);
  if (it != dbcs.end()) {
    return it->second;
  }
  DBC* dbc = dbc_parse(dbc_name);
  dbcs[dbc_name] = dbc;
  return dbc;
}

extern "C" {
const DBC* dbc_lookup(const char* dbc_name) {
  return dbc_lookup(std::string(dbc_name));
}
}
