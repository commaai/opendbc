#include <cassert>
#include <iostream>
#include "common_dbc.h"
#include "common.h"
#include "messaging.hpp"


extern "C" int LLVMFuzzerTestOneInput(const uint8_t *dat, size_t sz) {
	const size_t min_sz = 1 + 4 + 2;
	if (sz < min_sz || sz > min_sz + 8){
		return 0;
	}

	uint8_t src = dat[0];
	uint32_t addr = *(uint32_t*)&dat[1];
	uint16_t time = *(uint16_t*)&dat[5];
	size_t dat_sz = sz - min_sz;
	assert(dat_sz <= 8);

	auto parser = CANParser(0, "toyota_prius_2017_pt_generated",
			{{37, 1}, {740, 1}},
			{{37, "STEER_ANGLE", 0}, {740, "STEER_REQUEST", 0}});

	MessageBuilder msg;

	auto event = msg.initEvent();
        auto canData = event.initCan(1);
        canData[0].setAddress(addr);
        canData[0].setBusTime(time);

	uint8_t data[8];
	memcpy(data, &dat[6], dat_sz);
	size_t len = dat_sz;

	canData[0].setDat(kj::arrayPtr((uint8_t*)&data[0], len));
	canData[0].setSrc(src);

	parser.UpdateCans(0, canData);
	return 0;
}
