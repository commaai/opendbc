#include "safety.h"
#include "board/utils.h"

// this file is checked by cppcheck

// Ignore misra-c2012-8.7 as these functions are only called from panda and libsafety
bool heartbeat_engaged __attribute__((unused));

bool safety_rx_hook(const CANPacket_t *to_push) UNUSED_FUNC;
bool safety_tx_hook(CANPacket_t *to_send) UNUSED_FUNC;
int safety_fwd_hook(int bus_num, int addr) UNUSED_FUNC;
void safety_tick(const safety_config *cfg) UNUSED_FUNC;
int set_safety_hooks(uint16_t mode, uint16_t param) UNUSED_FUNC;
