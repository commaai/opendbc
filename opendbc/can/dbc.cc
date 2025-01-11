#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <re2/re2.h>
#include <set>
#include <sstream>
#include <vector>
#include <mutex>
#include <iterator>
#include <cstring>
#include <clocale>

#include "opendbc/can/common.h"
#include "opendbc/can/common_dbc.h"

RE2 bo_regexp(R"(^BO_ (\w+) (\w+) *: (\w+) (\w+))");
RE2 sg_regexp(R"(^SG_ (\w+) .*: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))");
RE2 val_regexp(R"(VAL_ (\w+) (\w+) (.*))");
RE2 val_split_regexp(R"((([0-9]) \"(.+?)\"))");

#define DBC_ASSERT(condition, message)                             \
  do {                                                             \
    if (!(condition)) {                                            \
      std::stringstream is;                                        \
      is << "[" << dbc_name << ":" << line_num << "] " << message; \
      throw std::runtime_error(is.str());                          \
    }                                                              \
  } while (false)

inline bool startswith(const std::string& str, const char* prefix) {
  return str.find(prefix, 0) == 0;
}

inline bool startswith(const std::string& str, std::initializer_list<const char*> prefix_list) {
  for (auto prefix : prefix_list) {
    if (startswith(str, prefix)) return true;
  }
  return false;
}

inline bool endswith(const std::string& str, const char* suffix) {
  return str.find(suffix, 0) == (str.length() - strlen(suffix));
}

inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v") {
  s.erase(s.find_last_not_of(t) + 1);
  return s.erase(0, s.find_first_not_of(t));
}

ChecksumState* get_checksum(const std::string& dbc_name) {
  ChecksumState* s = nullptr;
  if (startswith(dbc_name, {"honda_", "acura_"})) {
    s = new ChecksumState({4, 2, 3, 5, false, HONDA_CHECKSUM, &honda_checksum});
  } else if (startswith(dbc_name, {"toyota_", "lexus_"})) {
    s = new ChecksumState({8, -1, 7, -1, false, TOYOTA_CHECKSUM, &toyota_checksum});
  } else if (startswith(dbc_name, "hyundai_canfd")) {
    s = new ChecksumState({16, -1, 0, -1, true, HKG_CAN_FD_CHECKSUM, &hkg_can_fd_checksum});
  } else if (startswith(dbc_name, {"vw_mqb_2010", "vw_mqbevo"})) {
    s = new ChecksumState({8, 4, 0, 0, true, VOLKSWAGEN_MQB_CHECKSUM, &volkswagen_mqb_checksum});
  } else if (startswith(dbc_name, "vw_golf_mk4")) {
    s = new ChecksumState({8, 4, 0, -1, true, XOR_CHECKSUM, &xor_checksum});
  } else if (startswith(dbc_name, "subaru_global_")) {
    s = new ChecksumState({8, -1, 0, -1, true, SUBARU_CHECKSUM, &subaru_checksum});
  } else if (startswith(dbc_name, "chrysler_")) {
    s = new ChecksumState({8, -1, 7, -1, false, CHRYSLER_CHECKSUM, &chrysler_checksum});
  } else if (startswith(dbc_name, "fca_giorgio")) {
    s = new ChecksumState({8, -1, 7, -1, false, FCA_GIORGIO_CHECKSUM, &fca_giorgio_checksum});
  } else if (startswith(dbc_name, "comma_body")) {
    s = new ChecksumState({8, 4, 7, 3, false, PEDAL_CHECKSUM, &pedal_checksum});
  }
  return s;
}

void set_signal_type(Signal& s, ChecksumState* chk, const std::string& dbc_name, int line_num) {
  s.calc_checksum = nullptr;
  if (chk) {
    if (s.name == "CHECKSUM") {
      DBC_ASSERT(chk->checksum_size == -1 || s.size == chk->checksum_size, "CHECKSUM is not " << chk->checksum_size << " bits long");
      DBC_ASSERT(chk->checksum_start_bit == -1 || (s.start_bit % 8) == chk->checksum_start_bit, " CHECKSUM starts at wrong bit");
      DBC_ASSERT(s.is_little_endian == chk->little_endian, "CHECKSUM has wrong endianness");
      DBC_ASSERT(chk->calc_checksum != nullptr, "CHECKSUM calculate function not supplied");
      s.type = chk->checksum_type;
      s.calc_checksum = chk->calc_checksum;
    } else if (s.name == "COUNTER") {
      DBC_ASSERT(chk->counter_size == -1 || s.size == chk->counter_size, "COUNTER is not " << chk->counter_size << " bits long");
      DBC_ASSERT(chk->counter_start_bit == -1 || (s.start_bit % 8) == chk->counter_start_bit, "COUNTER starts at wrong bit");
      DBC_ASSERT(chk->little_endian == s.is_little_endian, "COUNTER has wrong endianness");
      s.type = COUNTER;
    }
  }

  // TODO: CAN packer/parser shouldn't know anything about interceptors or pedals
  if (s.name == "CHECKSUM_PEDAL") {
    DBC_ASSERT(s.size == 8, "INTERCEPTOR CHECKSUM is not 8 bits long");
    s.type = PEDAL_CHECKSUM;
  } else if (s.name == "COUNTER_PEDAL") {
    DBC_ASSERT(s.size == 4, "INTERCEPTOR COUNTER is not 4 bits long");
    s.type = COUNTER;
  }
}

DBC* dbc_parse_from_stream(const std::string &dbc_name, std::istream &stream, ChecksumState *checksum, bool allow_duplicate_msg_name) {
  uint32_t address = 0;
  std::set<uint32_t> address_set;
  std::set<std::string> msg_name_set;
  std::map<uint32_t, std::set<std::string>> signal_name_sets;
  std::map<uint32_t, std::vector<Signal>> signals;
  DBC* dbc = new DBC;
  dbc->name = dbc_name;
  std::setlocale(LC_NUMERIC, "C");

  // used to find big endian LSB from MSB and size
  std::vector<int> be_bits;
  for (int i = 0; i < 64; i++) {
    for (int j = 7; j >= 0; j--) {
      be_bits.push_back(j + i * 8);
    }
  }

  std::string line;
  int line_num = 0;
  std::string match1, match2, match3, match4, match5, match6, match7;
  while (std::getline(stream, line)) {
    line = trim(line);
    line_num += 1;
    if (startswith(line, "BO_ ")) {
      // new group
      bool ret = RE2::FullMatch(line, bo_regexp, &match1, &match2, &match3);
      DBC_ASSERT(ret, "bad BO: " << line);

      Msg& msg = dbc->msgs.emplace_back();
      address = msg.address = std::stoul(match1);  // could be hex
      msg.name = match2;
      msg.size = std::stoul(match3);

      // check for duplicates
      DBC_ASSERT(address_set.find(address) == address_set.end(), "Duplicate message address: " << address << " (" << msg.name << ")");
      address_set.insert(address);

      if (!allow_duplicate_msg_name) {
        DBC_ASSERT(msg_name_set.find(msg.name) == msg_name_set.end(), "Duplicate message name: " << msg.name);
        msg_name_set.insert(msg.name);
      }
    } else if (startswith(line, "SG_ ")) {
      // new signal
      bool ret = RE2::FullMatch(line, sg_regexp, &match1, &match2, &match3, &match4, &match5, &match6, &match7);
      DBC_ASSERT(ret, "bad SG: " << line);
      Signal& sig = signals[address].emplace_back();
      sig.name = match1;
      sig.start_bit = std::stoi(match2);
      sig.size = std::stoi(match3);
      sig.is_little_endian = std::stoi(match4) == 1;
      sig.is_signed = match5 == "-";
      sig.factor = std::stod(match6);
      sig.offset = std::stod(match7);
      set_signal_type(sig, checksum, dbc_name, line_num);
      if (sig.is_little_endian) {
        sig.lsb = sig.start_bit;
        sig.msb = sig.start_bit + sig.size - 1;
      } else {
        auto it = find(be_bits.begin(), be_bits.end(), sig.start_bit);
        sig.lsb = be_bits[(it - be_bits.begin()) + sig.size - 1];
        sig.msb = sig.start_bit;
      }
      DBC_ASSERT(sig.lsb < (64 * 8) && sig.msb < (64 * 8), "Signal out of bounds: " << line);

      // Check for duplicate signal names
      DBC_ASSERT(signal_name_sets[address].find(sig.name) == signal_name_sets[address].end(), "Duplicate signal name: " << sig.name);
      signal_name_sets[address].insert(sig.name);
    } else if (startswith(line, "VAL_ ")) {
      // new signal value/definition
      bool ret = RE2::FullMatch(line, val_regexp, &match1, &match2, &match3);
      DBC_ASSERT(ret, "bad VAL: " << line);

      auto& val = dbc->vals.emplace_back();
      val.address = std::stoul(match1);  // could be hex
      val.name = match2;

      auto defvals = match3;
      // convert strings to UPPER_CASE_WITH_UNDERSCORES
      std::vector<std::string> words;
      std::string full_match, number, word;
      while (RE2::PartialMatch(defvals, val_split_regexp, &full_match, &number, &word)) {
        word = trim(word);
        std::transform(word.begin(), word.end(), word.begin(), ::toupper);
        std::replace(word.begin(), word.end(), ' ', '_');
        words.push_back(number + " " + word);
        defvals = defvals.substr(full_match.length(), defvals.length() - full_match.length());
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
    dbc->addr_to_msg[m.address] = &m;
    dbc->name_to_msg[m.name] = &m;
  }
  for (auto& v : dbc->vals) {
    v.sigs = signals[v.address];
  }
  return dbc;
}

DBC* dbc_parse(const std::string& dbc_path) {
  std::ifstream infile(dbc_path);
  if (!infile) return nullptr;

  const std::string dbc_name = std::filesystem::path(dbc_path).filename();

  std::unique_ptr<ChecksumState> checksum(get_checksum(dbc_name));
  return dbc_parse_from_stream(dbc_name, infile, checksum.get());
}

const std::string get_dbc_root_path() {
  char *basedir = std::getenv("BASEDIR");
  if (basedir != NULL) {
    return std::string(basedir) + "/opendbc/dbc";
  } else {
    return DBC_FILE_PATH;
  }
}

const DBC* dbc_lookup(const std::string& dbc_name) {
  static std::mutex lock;
  static std::map<std::string, DBC*> dbcs;

  std::string dbc_file_path = dbc_name;
  if (!std::filesystem::exists(dbc_file_path)) {
    dbc_file_path = get_dbc_root_path() + "/" + dbc_name + ".dbc";
  }

  std::unique_lock lk(lock);
  auto it = dbcs.find(dbc_name);
  if (it == dbcs.end()) {
    it = dbcs.insert(it, {dbc_name, dbc_parse(dbc_file_path)});
  }
  return it->second;
}

std::vector<std::string> get_dbc_names() {
  static const std::string& dbc_file_path = get_dbc_root_path();
  std::vector<std::string> dbcs;
  for (std::filesystem::directory_iterator i(dbc_file_path), end; i != end; i++) {
    if (!is_directory(i->path())) {
      std::string filename = i->path().filename();
      if (!startswith(filename, "_") && endswith(filename, ".dbc")) {
        dbcs.push_back(filename.substr(0, filename.length() - 4));
      }
    }
  }
  return dbcs;
}
