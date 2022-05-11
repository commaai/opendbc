#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <pwd.h>
#include <set>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <vector>

#include "selfdrive/common/timing.h"
#include "common_dbc.h"
#include "ctre.hpp"


std::string dbc_file_path;

#define DBC_ASSERT(condition, message)          \
  do {                                          \
    if (!(condition)) {                         \
      std::stringstream is;                     \
      is << "[" << dbc_name << "] " << message; \
      throw std::runtime_error(is.str());       \
    }                                           \
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
      DBC_ASSERT(s.size == chk->checksum_size, "CHECKSUM is not " << chk->checksum_size << " bits long");
      DBC_ASSERT((s.start_bit % 8) == chk->checksum_start_bit, " CHECKSUM starts at wrong bit");
      DBC_ASSERT(s.is_little_endian == chk->little_endian, "CHECKSUM has wrong endianness");
      s.type = chk->checksum_type;
    } else if (s.name == "COUNTER") {
      DBC_ASSERT(chk->counter_size == -1 || s.size == chk->counter_size, "COUNTER is not " << chk->counter_size << " bits long");
      DBC_ASSERT(chk->counter_start_bit == -1 || s.start_bit % 8 == chk->counter_start_bit, "COUNTER starts at wrong bit");
      DBC_ASSERT(chk->little_endian == s.is_little_endian, "COUNTER has wrong endianness");
      s.type = chk->counter_type;
    }
  }
  // TODO: replace hardcoded addresses with signal names. prefix with COMMA_PEDAL_?
  if (address == 0x200 || address == 0x201) {
    if (s.name == "CHECKSUM_PEDAL") {
      DBC_ASSERT(s.size == 8, "PEDAL CHECKSUM is not 8 bits long");
      s.type = PEDAL_CHECKSUM;
    } else if (s.name == "COUNTER_PEDAL") {
      DBC_ASSERT(s.size == 4, "PEDAL COUNTER is not 4 bits long");
      s.type = PEDAL_COUNTER;
    }
  } else if (address == 592) {
    if (s.name == "CHECKSUM") {
      DBC_ASSERT(s.size == 8, "BODY CHECKSUM is not 8 bits long");
      s.type = PEDAL_CHECKSUM;
    } else if (s.name == "COUNTER") {
      DBC_ASSERT(s.size == 4, "BODY COUNTER is not 4 bits long");
      s.type = PEDAL_COUNTER;
    }
  }
}

DBC* dbc_parse(const std::string& dbc_name) {
  double start_t = millis_since_boot();
  std::ifstream infile(dbc_file_path + "/" + dbc_name + ".dbc");
  if (!infile) return nullptr;

  static constexpr auto bo_regexp_ctre = ctll::fixed_string{ R"(^BO_ (\w+) (\w+) *: (\w+) (\w+))" };
  static constexpr auto sg_regexp_ctre = ctll::fixed_string{ R"(^SG_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))" };
  static constexpr auto sgm_regexp_ctre = ctll::fixed_string{ R"(^SG_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))" };
  static constexpr auto val_regexp_ctre = ctll::fixed_string{ R"(VAL_ (\w+) (\w+) (\s*[\-\+]?[0-9]+\s+\".+?\"[^;]*);)" };
  static constexpr auto val_split_regexp_ctre = ctll::fixed_string{ R"([\"]+)" };  // split on "
  printf("\nctre regex patterns: %lf ms\n", millis_since_boot() - start_t);

  std::unique_ptr<ChecksumState> checksum(get_checksum(dbc_name));

  uint32_t address = 0;
  std::set<uint32_t> address_set;
  std::set<std::string> msg_name_set;
  std::map<uint32_t, std::vector<Signal>> signals;
  DBC* dbc = new DBC;
  dbc->name = dbc_name;

  // used to find big endian LSB from MSB and size
  std::vector<int> be_bits;
  for (int i = 0; i < 64; i++) {
    for (int j = 7; j >= 0; j--) {
      be_bits.push_back(j + i * 8);
    }
  }

  start_t = millis_since_boot();
  std::string line;
  double tot_bo = 0;
  double tot_sg = 0;
  double tot_val = 0;
  double t = 0;
  while (std::getline(infile, line)) {
    line = trim(line);
    if (startswith(line, "BO_ ")) {
      t = millis_since_boot();
      // new group
      auto match = ctre::match<bo_regexp_ctre>(line);
      DBC_ASSERT(match, "bad BO %s" << line);

      Msg& msg = dbc->msgs.emplace_back();
      address = msg.address = match.get<1>().to_number();  // could be hex
      msg.name = match.get<2>().to_string();
      msg.size = std::stoul(match.get<3>().to_string());

      // check for duplicates
      DBC_ASSERT(address_set.find(address) == address_set.end(), "Duplicate address detected : " << address);
      address_set.insert(address);
      DBC_ASSERT(msg_name_set.find(msg.name) == msg_name_set.end(), "Duplicate message name : " << msg.name);
      msg_name_set.insert(msg.name);
      tot_bo += millis_since_boot() - t;
    } else if (startswith(line, "SG_ ")) {
      // new signal
      t = millis_since_boot();

      auto match = ctre::match<sg_regexp_ctre>(line);
      tot_sg += millis_since_boot() - t;
      Signal& sig = signals[address].emplace_back();
      sig.name = match.get<1>().to_string();

      if (match) {
        sig.start_bit = match.get<2>().to_number();
        sig.size = match.get<3>().to_number();
        sig.is_little_endian = match.get<4>().to_number() == 1;
        sig.is_signed = match.get<5>().to_string() == "-";
        sig.factor = std::stod(match.get<6>().to_string());
        sig.offset = std::stod(match.get<7>().to_string());
      } else {
        auto match_offset = ctre::match<sgm_regexp_ctre>(line);
        DBC_ASSERT(match_offset, "bad SG " << line);
        sig.start_bit = match_offset.get<3>().to_number();
        sig.size = match_offset.get<4>().to_number();
        sig.is_little_endian = match_offset.get<5>().to_number() == 1;
        sig.is_signed = match_offset.get<6>().to_string() == "-";
        sig.factor = std::stod(match_offset.get<7>().to_string());
        sig.offset = std::stod(match_offset.get<8>().to_string());
      }
      set_signal_type(sig, address, checksum.get(), dbc_name);

      if (sig.is_little_endian) {
        sig.lsb = sig.start_bit;
        sig.msb = sig.start_bit + sig.size - 1;
      } else {
        auto it = find(be_bits.begin(), be_bits.end(), sig.start_bit);
        sig.lsb = be_bits[(it - be_bits.begin()) + sig.size - 1];
        sig.msb = sig.start_bit;
      }
      DBC_ASSERT(sig.lsb < (64 * 8) && sig.msb < (64 * 8), "Signal out of bounds : " << line);
    } else if (startswith(line, "VAL_ ")) {
      t = millis_since_boot();
      // new signal value/definition
      auto match = ctre::match<val_regexp_ctre>(line);
      DBC_ASSERT(match, "bad VAL " << line);

      auto& val = dbc->vals.emplace_back();
      val.address = match.get<1>().to_number();  // could be hex
      val.name = match.get<2>().to_string();

      auto defvals = match.get<3>().to_string();
      // convert strings to UPPER_CASE_WITH_UNDERSCORES
      std::vector<std::string> words;
      for (auto word : ctre::split<val_split_regexp_ctre>(defvals)) {
        std::string w = word.to_string();
        w = trim(w);
        std::transform(w.begin(), w.end(), w.begin(), ::toupper);
        std::replace(w.begin(), w.end(), ' ', '_');
        words.push_back(w);
      }

      // join string
      std::stringstream s;
      std::copy(words.begin(), words.end(), std::ostream_iterator<std::string>(s, " "));
      val.def_val = s.str();
      val.def_val = trim(val.def_val);
      tot_val += millis_since_boot() - t;
    }
  }
  printf("\n\nloop time: %lf ms\n", millis_since_boot() - start_t);
  printf("bo: %lf ms\n", tot_bo);
  printf("sg: %lf ms\n", tot_sg);
  printf("val:  %lf ms\n", tot_val);

  for (auto& m : dbc->msgs) {
    m.sigs = signals[m.address];
  }
  for (auto& v : dbc->vals) {
    v.sigs = signals[v.address];
  }
  printf("\ntotal dbc_parse time: %lf ms\n", millis_since_boot() - start_t);
  return dbc;
}

void set_dbc_file_path() {
  if (dbc_file_path.empty()) {
    char *basedir = std::getenv("BASEDIR");
    if (basedir != NULL) {
      dbc_file_path = std::string(basedir) + "/opendbc";
    } else {
      dbc_file_path = DBC_FILE_PATH;
    }
  }
}

const DBC* dbc_lookup(const std::string& dbc_name) {
  double start_t = millis_since_boot();
  static std::mutex lock;
  static std::map<std::string, DBC*> dbcs;

  std::unique_lock lk(lock);
  set_dbc_file_path();

  auto it = dbcs.find(dbc_name);
  if (it == dbcs.end()) {
    it = dbcs.insert(it, {dbc_name, dbc_parse(dbc_name)});
  }
  printf("total dbc_lookup time: %lf ms\n\n", millis_since_boot() - start_t);
  return it->second;
}

std::vector<std::string> get_dbc_names() {
  set_dbc_file_path();
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
