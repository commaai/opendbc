import math
from enum import Enum

from opendbc.car import CarSpecs, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.docs_definitions import CarDocs, CUSTOM_CAR_PARTS, CarFootnote, Column
from opendbc.car.lateral import AngleSteeringLimits


# --- CAN buses ---
class CANBUS:
  main_bus = 0   # PT / EPS  — LANE_KEEP + LKAS_INFO TX
  cam_bus = 2    # stock cam — HUD, LANE_KEEP RX


# --- CarController timing ---
LANE_KEEP_STEP = 2   # 50 Hz @ DT_CTRL=0.01
LKAS_INFO_STEP = 2   # 50 Hz — match stock LKAS_INFO on bus 2
HUD_STEP = 5         # 20 Hz — match stock HUD parser rate
EPS_SPOOF_STEP = 1   # 100 Hz — match stock EPS rate

# EPS DRIVER_TORQUE pass-through + periodic "tap" injection.
# Stock distribution measured on this car (route 2026-05-26--04-52-57):
#   standstill, hands off, cruise off:  T=0 in 99% of frames
#   moving, hands off, cruise off:      T=0 in 61% of frames, 1..5 in 32%
#   moving, hands on (light grip):      T=10..15 typical, p99=15
#   moving, hands on (override):        T>=80 spikes briefly
# Any constant non-zero value while the wheel isn't moving reads as "fault" to the cam.
# Strategy: mirror real DRIVER_TORQUE exactly, and only when LKAS is asking for steering
# inject a brief light-touch tap every few seconds to keep the HOW timer reset.
EPS_TAP_TORQUE = 12            # mid of stock "light grip" band (10..15)
EPS_TAP_FRAMES = 8             # ~80 ms tap at 100 Hz
EPS_TAP_PERIOD_FRAMES = 400    # ~4 s between taps

# 10 Hz lowpass on the OP angle command (barely filters at 50 Hz LANE_KEEP).
STEER_LOWPASS_ALPHA = math.exp(-2.0 * math.pi * 10.0 * 0.02)

# Auto-resume from standstill alternates one RES burst then one SET burst:
# stock RES bumps set-speed by +1 km/h, so SET cancels that drift.
AUTORESUME_CYCLE_S = 1.2
AUTORESUME_BURST_FRAMES = 4
OMODA_PCM_DISABLE_RES_CYCLE_S = 3.5


# --- CarState ---
GEAR_MAP = {1: "P", 2: "R", 3: "N", 4: "D"}
OMODA_GEAR_MAP = {0xB: "D", 0xC: "N", 0xD: "R", 0xE: "P"}
# iCaur 0x315 gear nibble: P=11 D=1 N=10 R=9
ICAUR_GEAR_MAP = {11: "P", 1: "D", 10: "N", 9: "R"}
# iCaur 0x245 blinker nibble
ICAUR_BLINKER_LEFT = (4, 6)
ICAUR_BLINKER_RIGHT = (8, 9)
# HUD FOLLOW_DISTANCE: raw 1 = 1-bar (closest) … raw 5 = 5-bar (farthest); 0/6/7 unknown.
FOLLOW_RAW_TO_PERSONALITY = {1: 0, 2: 0, 3: 1, 4: 2, 5: 2}  # 0 aggressive / 1 standard / 2 relaxed
STEER_RELATED_INTERVENTION_RAW_MIN = 36000
# Jaecoo only: STEER_RELATED status when raw>=36000 (decoded with STEERING_ANGLE factor/offset).
# iCaur must not use this — 0xC4 STEERING_ANGLE is real road angle there.
STEER_RELATED_INTERVENTION_DEG_MIN = 36000 * 0.06 - 1966

PT_PARSER_MSGS = [
  ("WHEELSPEED_1", 50), ("WHEELSPEED_2", 50), ("EPS", 100), ("GAS", 100),
  ("TRANSMISSION", 100), ("BRAKE_PEDAL", 50), ("STALK", 50), ("PCM_BUTTONS", 20),
  ("ADAS_RELATED", 100), ("SPEED_RELATED", 50), ("STEER_RELATED", 100),
  ("SEATBELT_287", 50), ("SEATBELT_430", 50), ("BCM_STAT_412", math.nan),
  ("BCM_STAT_465", math.nan), ("LKAS_INFO", 50),
]
CAM_PARSER_MSGS = [("HUD", 20), ("LANE_KEEP", 50), ("ACC_UNCERTAIN", 20)]
# iCaur PT: only bus-0 signals CarState reads. Do NOT require HUD/LANE_KEEP on PT —
# HUD now forwards natively (Omoda-style); LANE_KEEP is still blocked cam->PT and
# re-emitted by CarController. Cam ADAS (HUD/LANE_KEEP) stay on ICAUR_CAM for parse.
ICAUR_PT_PARSER_MSGS = [
  ("ICAUR_WHEELSPEED_A", 50), ("ICAUR_WHEELSPEED_B", 50),
  ("ICAUR_BRAKE", 50), ("ICAUR_GAS", 50),
  ("STEER_RELATED", 100),
  ("ICAUR_STALK", 50), ("ICAUR_TRANSMISSION", 100),
]
ICAUR_CAM_PARSER_MSGS = [("HUD", 20), ("LANE_KEEP", 50), ("ACC_UNCERTAIN", 20)]
# iCaur03 pedal decode:
# - BRAKE: 0x132 start bit 31 (8-bit); DBC scale 0.01 → raw 100 = 1.0
# - GAS:   0x14A start bit 23 (12-bit); DBC (0.001,-2) → (raw-2000)/1000; idle 2000→0, 3000→1
ICAUR_BRAKE_PRESSED = 0.05  # scaled 0..1
ICAUR_GAS_PRESSED = 0.05
OMODA_PT_PARSER_MSGS = [
  ("WHEELSPEED_1", 50), ("WHEELSPEED_2", 50), ("EPS", 100), ("GAS", 100),
  ("STALK", 50), ("PCM_BUTTONS", 20),
  ("ADAS_RELATED", 100), ("SPEED_RELATED", 50), ("STEER_RELATED", 100),
  ("SEATBELT_287", 50), ("SEATBELT_430", 50), ("BCM_STAT_412", math.nan),
  ("BCM_STAT_465", math.nan), ("LKAS_INFO", 50),
  ("OMODA_TRANSMISSION", 100), ("OMODA_BRAKE", 50), ("HUD", 20),
]

# Omoda 5: 0x2E9 byte 2 is multiplexed — raw 1..5 are message variants, not pedal.
OMODA_BRAKE_PRESSURE_RAW_MAX = 0x35
OMODA_BRAKE_PRESSURE_RAW_MIN = 5

# Omoda 5: torque spoof + HUD override temporarily off (meter errors when enabled).
OMODA_DISABLE_TORQUE_SPOOF = True
OMODA_DISABLE_HUD_OVERRIDE = True
CHERY_OMODA_SAFETY_PARAM = 1
CHERY_OMODA_NO_TORQUE_SPOOF_PARAM = 2
# iCaur 03: standstill on 0x222; torque spoof off (same bit as Omoda).
# HUD: Omoda-style — native cam HUD forwards to PT; no HUD override TX.
CHERY_ICAUR_SAFETY_PARAM = 4
ICAUR_DISABLE_TORQUE_SPOOF = True
ICAUR_DISABLE_HUD_OVERRIDE = True
# iCaur steer override: latch (drop LKAS) on steeringPressed, firm yank, OR opposing torque.
# OPPOSING_TORQUE=10 matches CarState steeringPressed floor (Jul20-21 qlogs: pressed p50=23,
# sharp-turn |T|>=10 is 4.2% vs 3.4% pressed; quiet p99=2). CMD_DEADBAND=0.5° ignores noise
# when planner angle ≈ measured. Exit: |T|<=3 for 20 frames (~200 ms).
# PRESSED_MIN_COUNT: was 5 (~50ms+). 0 ⇒ count>0 on first |T|>=10 frame (~10ms) so the
# first intentional tug drops LKAS; accepts ~+3pp sharp-turn latch vs debounce=5 (Jul30 sim).
# INSTANT_TORQUE: redundant firm-yank path (frame-1 |T|>=20) if pressed publish lags.
ICAUR_DRIVER_OVERRIDE_ENABLED = True
ICAUR_OVERRIDE_OPPOSING_TORQUE = 10
ICAUR_STEERING_PRESSED_TORQUE = 10
ICAUR_STEERING_PRESSED_MIN_COUNT = 0
ICAUR_OVERRIDE_INSTANT_TORQUE = 20
ICAUR_LKAS_CMD_DEADBAND_DEG = 0.5
ICAUR_OVERRIDE_EXIT = 3
ICAUR_OVERRIDE_EXIT_FRAMES = 20
OMODA_CAM_PARSER_MSGS = [("STEER_STATUS", 20), ("LANE_KEEP", 50), ("ACC_UNCERTAIN", 20)]


# --- cherycan TX constants ---
LANE_KEEP_PADDING = {
  "SET_ME_XFF": 255, "SET_ME_XFC": 252, "SET_ME_XF4": 244, "SET_ME_X63": 99, "SET_ME_XF": 15,
}

# LKAS torque spoof params (tuned to stock MAIN_TORQUE distribution: p10=7, p50=63, p90=149).
SPOOF_TORQUE_MIN = 30.0
SPOOF_TORQUE_VAR_MIN = 25.0
SPOOF_TORQUE_RAMP = 8.0
SPOOF_TORQUE_MAX = 110.0
SPOOF_NEG_PROB = 0.3
SPOOF_VAR_PROB = 0.2


def lowpass_steer_cmd(x: float, y_prev: float | None) -> float:
  if y_prev is None:
    return x
  return STEER_LOWPASS_ALPHA * y_prev + (1.0 - STEER_LOWPASS_ALPHA) * x


class CarControllerParams:
  STEER_ANGLE_MAX = 120.0
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[50.0, 40.0, 25.0])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[60.0, 50.0, 30.0])
  ANGLE_LIMITS = AngleSteeringLimits(STEER_ANGLE_MAX, ANGLE_RATE_LIMIT_UP, ANGLE_RATE_LIMIT_DOWN)


# --- Docs / platform ---
class Footnote(Enum):
  J7_NOTE = CarFootnote(
    "Support: under validation on chery_general_pt.dbc (Jaecoo J7 PHEV).",
    Column.LONGITUDINAL,
  )


class CAR(Platforms):
  CHERY_JAECOO_J7_PHEV = PlatformConfig(
    [CarDocs(
      "Jaecoo J7 PHEV 2024-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1980.0, wheelbase=2.67, steerRatio=16.0),
    dbc_dict("chery_general_pt", None),
  )
  CHERY_TIGGO_8_PRO = PlatformConfig(
    [CarDocs(
      "Chery Tiggo 8 Pro 2024-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1980.0, wheelbase=2.67, steerRatio=16.0),
    dbc_dict("chery_general_pt", None),
  )
  CHERY_OMODA_5 = PlatformConfig(
    [CarDocs(
      "Chery Omoda 5 2022-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1420.0, wheelbase=2.63, steerRatio=16.0),
    dbc_dict("chery_general_pt", None),
  )
  CHERY_ICAUR_03 = PlatformConfig(
    [CarDocs(
      "iCaur 03 2024-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1760.0, wheelbase=2.71, steerRatio=16.0),  # steerRatio matches Jaecoo J7  # steerRatio matches Jaecoo J7
    dbc_dict("chery_general_pt", None),
  )


DBC = CAR.create_dbc_map()
