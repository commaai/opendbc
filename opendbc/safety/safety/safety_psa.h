#define PSA_DRIVER                1390 // RX from BSI, gas pedal
#define PSA_DAT_BSI               1042 // RX from BSI, doors
#define PSA_HS2_DYN_ABR_38D       909  // RX from CAN1, UC_FREIN, speed
#define PSA_HS2_DAT_MDD_CMD_452   1106 // RX from CAN1, BSI, cruise state
#define PSA_LANE_KEEP_ASSIST      1010 // TX from OP, EPS
// RADAR
#define PSA_Req_Diag_ARTIV        1718 // TX from OP, radar diagnostics
#define PSA_HS2_SUPV_ARTIV_796    1942 // TX from OP, radar emulation
#define PSA_HS2_DAT_ARTIV_V2_4F6  1270 // TX from OP, radar emulation
#define PSA_HS2_DYN1_MDD_ETAT_2B6 694  // TX from OP, radar emulation
#define PSA_HS2_DYN_MDD_ETAT_2F6  758  // TX from OP, radar emulation

// CAN bus numbers
#define PSA_MAIN_BUS 2U
#define PSA_ADAS_BUS 1U
#define PSA_CAM_BUS  0U

const CanMsg PSA_TX_MSGS[] = {
  {PSA_LANE_KEEP_ASSIST, PSA_CAM_BUS, 8, true},
  {PSA_Req_Diag_ARTIV, PSA_ADAS_BUS, 8, false}, // TODO: check if reduce to 3 is ok
  {PSA_HS2_SUPV_ARTIV_796, PSA_ADAS_BUS, 8, false},
  {PSA_HS2_DAT_ARTIV_V2_4F6, PSA_ADAS_BUS, 5, false},
  {PSA_HS2_DYN1_MDD_ETAT_2B6, PSA_ADAS_BUS, 8, false},
  {PSA_HS2_DYN_MDD_ETAT_2F6, PSA_ADAS_BUS, 8, false},
};

RxCheck psa_rx_checks[] = {
  // TODO: counters and checksums
  {.msg = {{PSA_DRIVER, PSA_MAIN_BUS, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}}, // no counter
  {.msg = {{PSA_DAT_BSI, PSA_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 20U}, { 0 }, { 0 }}}, // no counter
  {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_ADAS_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 25U}, { 0 }, { 0 }}},
  {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 20U}, { 0 }, { 0 }}},
};

static bool psa_lkas_msg_check(int addr) {
  return addr == PSA_LANE_KEEP_ASSIST;
}

// TODO: update rate limits
// Currently set to ISO11270 limits
const AngleSteeringLimits PSA_STEERING_LIMITS = {
    .angle_deg_to_can = 100,
    .angle_rate_up_lookup = {
    {0., 5., 15.},
    {2.5, 1.5, 0.2},
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {5., 2.0, 0.3},
  },
};

static void psa_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == PSA_CAM_BUS) {
    if (addr == PSA_DAT_BSI) {
      brake_pressed = GET_BIT(to_push, 5); // P013_MainBrake
    }
    if (addr == PSA_DRIVER) {
      gas_pressed = GET_BYTE(to_push, 3) > 0U; // GAS_PEDAL
    }
    bool stock_ecu_detected = psa_lkas_msg_check(addr);
    generic_rx_checks(stock_ecu_detected);
  }
  if (bus == PSA_ADAS_BUS) {
    if (addr == PSA_HS2_DYN_ABR_38D) {
      int speed = (GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1);
      vehicle_moving = speed > 0;
      UPDATE_VEHICLE_SPEED(speed * 0.01); // VITESSE_VEHICULE_ROUES
    }
    if (addr == PSA_HS2_DAT_MDD_CMD_452) {
      pcm_cruise_check(GET_BIT(to_push, 23)); // DDE_ACTIVATION_RVV_ACC
    }
  }
}

static bool psa_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);

  // TODO: Safety check for cruise buttons
  // TODO: check resume is not pressed when controls not allowed
  // TODO: check cancel is not pressed when cruise isn't engaged

  // Safety check for LKA
  if (addr == PSA_LANE_KEEP_ASSIST) {
    // Signal: ANGLE
    int desired_angle = to_signed((GET_BYTE(to_send, 6) << 6) | ((GET_BYTE(to_send, 7) & 0xFCU) >> 2), 14);
    // Signal: TORQUE_FACTOR
    bool lka_active = ((GET_BYTE(to_send, 5) & 0xFEU) >> 1) == 100U;

    if (steer_angle_cmd_checks(desired_angle, lka_active, PSA_STEERING_LIMITS)) {
      tx = false;
    }
  }
  return tx;
}

static bool psa_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == PSA_MAIN_BUS) {
    block_msg = psa_lkas_msg_check(addr);
  }

  return block_msg;
}

static safety_config psa_init(uint16_t param) {
  UNUSED(param);
  print("psa_init\n");
  return BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .fwd = psa_fwd_hook,
  // .get_counter = psa_get_counter,
  // .get_checksums = psa_get_checksum,
};
