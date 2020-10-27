#include <cassert>
#include <iostream>
#include "common_dbc.h"
#include "common.h"
#include "messaging.hpp"


extern "C" int LLVMFuzzerTestOneInput(const uint8_t *dat, size_t sz) {
	const size_t min_sz = 1 + 4 + 2;
	if (sz < min_sz || sz > 8 * 128){
		return 0;
	}

	uint8_t src = dat[0];
	uint32_t addr = *(uint32_t*)&dat[1];
	uint16_t time = *(uint16_t*)&dat[5];
	uint8_t num_msg = (sz - min_sz) / 8 + 1;


	auto parser = CANParser(0, "toyota_prius_2017_pt_generated",
			{{37, 1}, {740, 1}},
			{{37, "STEER_ANGLE", 0}, {740, "STEER_REQUEST", 0}});

	auto parser2 = CANParser(0, "honda_civic_touring_2016_can_generated",
                          {{330, 1}},
                          {{330, "STEER_ANGLE", 0}});
  // TODO: add all brands

	MessageBuilder msg;

	auto event = msg.initEvent();
  auto canData = event.initCan(num_msg);

  size_t j = 0;
  for (size_t i = 6; i < sz; i += 8){
    size_t dat_sz = std::min(size_t(8), size_t(sz) - i - 1);
    canData[j].setAddress(addr);
    canData[j].setBusTime(time);

    canData[j].setDat(kj::arrayPtr(dat + i, dat_sz));
    canData[j].setSrc(src);
    j++;
  }

	parser.UpdateCans(0, canData);
	parser2.UpdateCans(0, canData);
	return 0;
}
