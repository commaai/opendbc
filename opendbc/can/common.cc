#include <array>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "opendbc/can/common.h"

void pedal_setup_signal(Signal &sig, const std::string& dbc_name, int line_num) {
  if (sig.name == "CHECKSUM_PEDAL") {
    DBC_ASSERT(sig.size == 8, "INTERCEPTOR CHECKSUM is not 8 bits long");
    sig.type = PEDAL_CHECKSUM;
  } else if (sig.name == "COUNTER_PEDAL") {
    DBC_ASSERT(sig.size == 4, "INTERCEPTOR COUNTER is not 4 bits long");
    sig.type = COUNTER;
  }
}

void tesla_setup_signal(Signal &sig, const std::string& dbc_name, int line_num) {
  if (endswith(sig.name, "Counter")) {
    sig.type = COUNTER;
  } else if (endswith(sig.name, "Checksum")) {
    sig.type = TESLA_CHECKSUM;
    sig.calc_checksum = &tesla_checksum;
  }
}
