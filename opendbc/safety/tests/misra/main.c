#include "opendbc/safety/safety.h"

// this file is checked by cppcheck

extern uint32_t microsecond_timer_get(void);

// Ignore misra-c2012-8.7 as these functions are only called from libsafety
SAFETY_UNUSED(safety_heartbeat_engaged_internal);

SAFETY_UNUSED(safety_rx_hook);
SAFETY_UNUSED(safety_tx_hook);
SAFETY_UNUSED(safety_fwd_hook);
SAFETY_UNUSED(safety_tick);
SAFETY_UNUSED(safety_get_state);
SAFETY_UNUSED(safety_disengage);
SAFETY_UNUSED(safety_set_alternative_experience);
SAFETY_UNUSED(safety_set_heartbeat_engaged);
SAFETY_UNUSED(set_safety_hooks);
