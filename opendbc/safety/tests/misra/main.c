#include "opendbc/safety/safety.h"

// this file is checked by cppcheck

extern uint32_t microsecond_timer_get(void);
extern uint32_t can_rx_frames_queued(void);
extern uint32_t can_rx_frames_read(void);

// Ignore misra-c2012-8.7 as these functions are only called from libsafety
SAFETY_UNUSED(safety.heartbeat_engaged);

SAFETY_UNUSED(safety_rx_hook);
SAFETY_UNUSED(safety_tx_hook);
SAFETY_UNUSED(safety_fwd_hook);
SAFETY_UNUSED(safety_tick);
SAFETY_UNUSED(safety_heartbeat);
SAFETY_UNUSED(safety_heartbeat_check);
SAFETY_UNUSED(safety_alternative_experience);
SAFETY_UNUSED(set_safety_hooks);
