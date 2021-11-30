#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <map>
#include <memory>
#include <regex>
#include <set>

#include "common.h"
#include "common_dbc.h"

namespace {

#define DBC_ASSERT(condition, message)          \
  do {                                          \
    if (!(condition)) {                         \
      std::stringstream is;                     \
      is << "[" << dbc_name << "] " << message; \
      throw std::runtime_error(is.str());       \
    }                                           \
  } while (false)

inline bool startswith(const std::string& str, const char* prefix) {
  return str.rfind(prefix, 0) == 0;
}

inline bool startswith(const std::string& str, std::initializer_list<const char*> prefix_list) {
  for (auto prefix : prefix_list) {
    if (startswith(str, prefix)) return true;
  }
  return false;
}

inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v") {
  s.erase(s.find_last_not_of(t) + 1);
  return s.erase(0, s.find_first_not_of(t));
}

typedef struct ChecksumState {
  int checksum_size;
  int counter_size;
  int checksum_start_bit;
  int counter_start_bit;
  bool little_endian;
  SignalType checksum_type;
  SignalType counter_type;
} ChecksumState;

ChecksumState* get_checksum(const std::string& dbc_name) {
  ChecksumState* s = nullptr;
  if (startswith(dbc_name, {"honda_", "acura_"})) {
    s = new ChecksumState({4, 2, 3, 5, false, HONDA_CHECKSUM, HONDA_COUNTER});
  } else if (startswith(dbc_name, {"toyota_", "lexus_"})) {
    s = new ChecksumState({8, -1, 7, -1, false, TOYOTA_CHECKSUM});
  } else if (startswith(dbc_name, {"vw_", "volkswagen_", "audi_", "seat_", "skoda_"})) {
    s = new ChecksumState({8, 4, 0, 0, true, VOLKSWAGEN_CHECKSUM, VOLKSWAGEN_COUNTER});
  } else if (startswith(dbc_name, "subaru_global_")) {
    s = new ChecksumState({8, -1, 0, -1, true, SUBARU_CHECKSUM});
  } else if (startswith(dbc_name, "chrysler_")) {
    s = new ChecksumState({8, -1, 7, -1, false, CHRYSLER_CHECKSUM});
  }
  return s;
}

void set_signal_type(Signal& s, uint32_t address, ChecksumState* chk, const std::string& dbc_name) {
  if (chk) {
    if (s.name == "CHECKSUM") {
      DBC_ASSERT(s.b2 == chk->checksum_size, "CHECKSUM is not " << chk->checksum_size << " bits long");
      DBC_ASSERT((s.b1 % 8) == chk->checksum_start_bit, " CHECKSUM starts at wrong bit");
      DBC_ASSERT(s.is_little_endian == chk->little_endian, "CHECKSUM has wrong endianness");
      s.type = chk->checksum_type;
    } else if (s.name == "COUNTER") {
      DBC_ASSERT(chk->counter_size == -1 || s.b2 == chk->counter_size, "COUNTER is not " << chk->counter_size << " bits long");
      DBC_ASSERT(chk->counter_start_bit == -1 || s.b1 % 8 == chk->counter_start_bit, "COUNTER starts at wrong bit");
      DBC_ASSERT(chk->little_endian == s.is_little_endian, "COUNTER has wrong endianness");
      s.type = chk->counter_type;
    }
  }
  // TODO: replace hardcoded addresses with signal names. prefix with COMMA_PEDAL_?
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
  std::ifstream infile(std::string(DBC_FILE_PATH) + "/" + dbc_name + ".dbc");
  if (!infile) return nullptr;

  std::regex bo_regexp(R"(^BO\_ (\w+) (\w+) *: (\w+) (\w+))");
  std::regex sg_regexp(R"(^SG\_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))");
  std::regex sgm_regexp(R"(^SG\_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))");
  std::regex val_regexp(R"(VAL\_ (\w+) (\w+) (\s*[-+]?[0-9]+\s+\".+?\"[^;]*))");

  std::unique_ptr<ChecksumState> checksum(get_checksum(dbc_name));

  uint32_t address = 0;
  std::set<uint32_t> address_set;
  std::set<std::string> msg_name_set;
  std::map<uint32_t, std::vector<Signal>> signals;
  DBC* dbc = new DBC;
  dbc->name = dbc_name;

  std::string line;
  while (std::getline(infile, line)) {
    line = trim(line);
    std::smatch match;
    if (startswith(line, "BO_ ")) {
      // new group
      bool ret = std::regex_match(line, match, bo_regexp);
      DBC_ASSERT(ret, "bad BO %s" << line);

      Msg& msg = dbc->msgs.emplace_back();
      address = msg.address = std::stoul(match[1].str());  // could be hex
      msg.name = match[2].str();
      msg.size = std::stoul(match[3].str());

      // check for duplicates
      DBC_ASSERT(address_set.find(address) == address_set.end(), "Duplicate address detected : " << address);
      address_set.insert(address);
      DBC_ASSERT(msg_name_set.find(msg.name) == msg_name_set.end(), "Duplicate message name : " << msg.name);
      msg_name_set.insert(msg.name);
    } else if (startswith(line, "SG_ ")) {
      // new signal
      int offset = 0;
      if (!std::regex_search(line, match, sg_regexp)) {
        bool ret = std::regex_search(line, match, sgm_regexp);
        DBC_ASSERT(ret, "bad SG " << line);
        offset = 1;
      }
      Signal& sig = signals[address].emplace_back();
      sig.name = match[1].str();
      sig.b1 = std::stoi(match[offset + 2].str());
      sig.b2 = std::stoi(match[offset + 3].str());
      sig.is_little_endian = std::stoi(match[offset + 4].str()) == 1;
      sig.is_signed = match[offset + 5].str() == "-";
      sig.factor = std::stod(match[offset + 6].str());
      sig.offset = std::stod(match[offset + 7].str());
      set_signal_type(sig, address, checksum.get(), dbc_name);
      if (!sig.is_little_endian) {
        uint64_t b1 = sig.b1;
        sig.b1 = std::floor(b1 / 8) * 8 + (-b1 - 1) % 8;
      }
      sig.bo = 64 - (sig.b1 + sig.b2);
    } else if (startswith(line, "VAL_ ")) {
      // new signal value/definition
      bool ret = std::regex_search(line, match, val_regexp);
      DBC_ASSERT(ret, "bad VAL " << line);

      auto& val = dbc->vals.emplace_back();
      val.address = std::stoul(match[1].str());  // could be hex
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
      val.def_val = trim(val.def_val);
    }
  }

  for (auto& m : dbc->msgs) {
    m.sigs = signals[m.address];
  }
  for (auto& v : dbc->vals) {
    v.sigs = signals[v.address];
  }
  return dbc;
}

}  // namespace

const DBC* dbc_lookup(const std::string& dbc_name) {
  static std::mutex lock;
  static std::map<std::string, DBC*> dbcs;

  std::unique_lock lk(lock);
  auto it = dbcs.find(dbc_name);
  if (it == dbcs.end()) {
    it = dbcs.insert(it, {dbc_name, dbc_parse(dbc_name)});
  }
  return it->second;
}
