from dataclasses import dataclass, field
from enum import IntFlag, StrEnum

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.lateral import AngleSteeringLimitsVM, ISO_LATERAL_ACCEL
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.vin import Vin

Ecu = structs.CarParams.Ecu


# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration


class BydFlags(IntFlag):
  ANGLE_CONTROL = 1
  EPS_ACK = 2            # standby needs REQ=0 / ACTIVE_LOW=0 ack
  LKS_HUD_CONTROL = 4    # bus-2 camera LKS toggle + 0x316 HUD hold


class CarControllerParams:
  # Mirrored in safety/modes/byd.h; class-level for safety tests
  STEER_STEP = 2  # 50 Hz angle command

  # LKS_PREPARED=1 when EPS idle; MAIN_TORQUE saturates near hard angle stops
  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    390,  # deg
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^3
    MAX_ANGLE_RATE=5,  # deg/20ms frame
  )

  ACCEL_MIN = -3.5                    # m/s^2
  ACCEL_MAX = 2.0                     # m/s^2
  GAS_RESUME_HARD = -1.0              # m/s^2, faster step at or below this
  GAS_RESUME_STEP = 0.05              # m/s^2 per 50 Hz slot
  GAS_RESUME_STEP_HARD = 0.10

  # Column torque; EPS DRIVER_EPS_TORQUE is attenuated ~12x and unusable for override
  STEER_DRIVER_OVERRIDE = 22          # Nm soft override
  STEER_DRIVER_DISENGAGE = 100        # Nm hard disengage
  STEER_DRIVER_DISENGAGE_FRAMES = 5   # 100 ms, mirrored in byd.h
  WHEELSPEED_TO_KPH = 0.072           # 0.02 m/s/LSB, mirrored in byd.h

  def __init__(self, CP):
    self.LOW_SPEED_RATE_BP = [0., 2.78, 5.56, 8.33]   # 0, 10, 20, 30 kph
    self.LOW_SPEED_RATE_V = [2., 2., 3., 5.]          # deg/20ms frame
    self.LOW_SPEED_TAU_BP = [0., 2.78, 5.56, 8.33]    # 0, 10, 20, 30 kph
    self.LOW_SPEED_TAU_V = [0.3, 0.3, 0.15, 0.]       # s

    self.STEER_NOT_ACCEPTED_FRAMES = 10      # 200 ms of 50 Hz STEER_REQ
    self.EPB_DEBOUNCE_FRAMES = 8             # ~70 ms at 120 Hz

    self.STEER_WARM_UP_FRAMES = 2            # 50 Hz; REQ=0 before first REQ=1
    self.STEER_YIELD_HOLD_SLOTS = 25         # 50 Hz, 0.5 s after driver torque

    self.STOCK_AEB_ACCEL = -1.5              # m/s^2
    self.STANDSTILL_RESUME_CLEAR_SPEED = 2.0 # m/s; stock clears RESUME here

    if CP.flags & BydFlags.EPS_ACK:
      self.STEER_ACK_PERIOD = 10               # 50 Hz between ack pairs
      self.STEER_ACK_ATTEMPTS = 3
      self.STEER_ACK_RETRY_PERIOD = 50         # 50 Hz, 1 s slow retry

    if CP.flags & BydFlags.LKS_HUD_CONTROL:
      self.LKS_PULSE_TICKS = 1                 # LKAS_ON_BTN frames per pulse
      self.LKS_PULSE_PERIOD = 5                # fallback if no fresh 0x3B0
      self.LKS_CONFIRM_FRAMES = 40             # 400 ms before one retry
      self.LKS_LOCKOUT_FRAMES = 50             # 500 ms after bus-0 press
      self.LKS_HUD_QUIET_FRAMES = 200          # 2 s, matches BYD_OP_HUD_TIMEOUT_US
      self.LKS_CAM_DEBOUNCE_FRAMES = 20        # 200 ms; treat state 4 as on
      self.LKS_EPS_IDLE_FRAMES = 20            # 200 ms PREP=1 CRUISE=0 before restore
      self.LKS_RECOVER_FRAMES = 200            # 2 s; retry failed want-off
      self.LKS_OFF_ALERT_FRAMES = 300          # 100 Hz, 3 s after SET/RES with LKS off


class BydSafetyFlags(IntFlag):
  LKS_ON = 2
  LONG_CONTROL = 4


class WMI(StrEnum):
  BYD_AUTO = "LGX"  # BYD Auto Co., Ltd. (Shenzhen)


class ModelYear(StrEnum):
  N_2022 = "N"
  P_2023 = "P"
  R_2024 = "R"
  S_2025 = "S"


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'byd_atto3',
    Bus.radar: 'byd_radar_fd',
  })
  wmis: set[WMI] = field(default_factory=set)
  years: set[ModelYear] = field(default_factory=set)

  def init(self):
    self.flags |= BydFlags.ANGLE_CONTROL | BydFlags.EPS_ACK | BydFlags.LKS_HUD_CONTROL


@dataclass(frozen=True)
class SafetyPlatformParams:
  # Mirrored in safety/modes/byd.h, selected by the safetyParam platform bit.
  # STEER_ANGLE_MAX is the EPS fault limit. ACCEL bounds feed byd_tx_hook.
  wheelbase: float
  steer_ratio: float
  steer_angle_max: float  # deg
  accel_min: float  # m/s^2
  accel_max: float  # m/s^2
  safety_param: int  # high-byte platform bit, BYD_PARAM_PLATFORM_*


SAFETY_PLATFORM_PARAMS: dict[str, SafetyPlatformParams] = {
  "BYD_ATTO_3": SafetyPlatformParams(
    wheelbase=2.72,
    steer_ratio=17.5,
    steer_angle_max=390.,  # deg, EPS fault limit
    accel_min=-3.5,
    accel_max=2.0,
    safety_param=0x100,  # BYD_PARAM_PLATFORM_ATTO3, 1U << 8
  ),
}


class CAR(Platforms):
  BYD_ATTO_3 = BydPlatformConfig(
    [BydCarDocs("BYD Atto 3 2022-25")],
    CarSpecs(wheelbase=2.72, steerRatio=17.5),
    wmis={WMI.BYD_AUTO},
    years={ModelYear.N_2022, ModelYear.P_2023, ModelYear.R_2024, ModelYear.S_2025},
  )


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  # BYD Atto 3 VIN: LGX (WMI) + <VDS> + <year><plant><seq> (VIS).
  # TODO: currently we only match on WMI + model year
  vin_obj = Vin(vin)
  year = vin_obj.vis[:1]

  candidates = set()
  for platform in CAR:
    if vin_obj.wmi in platform.config.wmis and year in platform.config.years:
      candidates.add(platform)

  return {str(c) for c in candidates}


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # BYD rejects MANUFACTURER_SOFTWARE_VERSION (0xF188). F195 is 6 raw bytes.
    Request(
      [StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
  non_essential_ecus={Ecu.fwdCamera: [CAR.BYD_ATTO_3]},
  fw_version_regex=rb"[\x00-\xff]{6}",
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)


DBC = CAR.create_dbc_map()
