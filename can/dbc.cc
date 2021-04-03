#include <vector>

#include "common_dbc.h"

static std::vector<DBC> dbcs;

const DBC* dbc_lookup(const std::string& dbc_name) {
  for (const auto& dbci : dbcs) {
    if (dbc_name == dbci.name) {
      return &dbci;
    }
  }
  return NULL;
}

void dbc_register(const DBC& dbc) {
  dbcs.push_back(dbc);
}

extern "C" {
  const DBC* dbc_lookup(const char* dbc_name) {
    return dbc_lookup(std::string(dbc_name));
  }
}
