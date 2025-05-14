#include <stdbool.h>

#include "safety/board/fake_stm.h"
#include "safety/board/can.h"

//int safety_tx_hook(CANPacket_t *to_send) { return 1; }

#include "safety/board/faults.h"
#include "safety/safety.h"
#include "safety/board/drivers/can_common.h"

// libsafety stuff
#include "safety/tests/libsafety/safety_helpers.h"
