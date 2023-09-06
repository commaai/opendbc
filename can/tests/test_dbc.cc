#include "opendbc/can/common.h"
#include "opendbc/can/common_dbc.h"
#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include "catch2/catch.hpp"

TEST_CASE("parse_dbc_file") {
  std::string content = R"(
BO_ 160 message_1: 8 EON
  SG_ signal_1 : 0|12@1+ (1,0) [0|4095] "unit"  XXX
  SG_ signal_2 : 12|1@1+ (1.0,0.0) [0.0|1] ""  XXX

VAL_ 160 signal_1 0 "disabled" 1.2 "initializing" 2 "fault";
)";

  std::istringstream stream(content);
  auto dbc = dbc_parse_from_stream("test_dbc", stream);
  REQUIRE(dbc != nullptr);

  std::map<uint32_t, Msg*> messages;
  for (auto &m : dbc->msgs) {
    messages[m.address] = &m;
  }

  auto msg = messages[160];
  REQUIRE(msg != nullptr);
  REQUIRE(msg->name == "message_1");
  REQUIRE(msg->size == 8);
  REQUIRE(msg->sigs.size() == 2);

  const auto &sig_1 = msg->sigs[0];
  REQUIRE(sig_1.name == "signal_1");
  REQUIRE(sig_1.start_bit == 0);
  REQUIRE(sig_1.size == 12);
  REQUIRE(sig_1.is_signed == false);
  REQUIRE(sig_1.factor == 1);
  REQUIRE(sig_1.offset == 0);
  REQUIRE(sig_1.is_little_endian == true);

  const auto &sig_2 = msg->sigs[1];
  REQUIRE(sig_2.name == "signal_2");

  std::map<std::pair<uint32_t, std::string>, Val *> vals;
  for (auto &v : dbc->vals) {
    vals[std::make_pair(v.address, v.name)] = &v;
  }
  auto val = vals[std::make_pair(160, "signal_1")];
  REQUIRE(val != nullptr);
  REQUIRE(val->name == "signal_1");
  REQUIRE(val->sigs.size() == 2);
  REQUIRE(val->sigs[0].name == "signal_1");
  REQUIRE(val->sigs[1].name == "signal_2");
  REQUIRE(val->def_val == "0 DISABLED 1.2 INITIALIZING 2 FAULT");
}
