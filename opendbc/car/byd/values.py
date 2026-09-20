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


class CarControllerParams:
  STEER_STEP = 2  # Angle command is sent at 50 Hz

  # LKS_PREPARED is inverted "EPS executing": 0 while steering for us, 1 when idle.
  # It goes 0->1 on any exit from steering (normal or fault), not only on a fault.
  # STEERING_TORQUE.MAIN_TORQUE is saturated at -300 for around 900ms,
  # while the wheel sits 15-26 deg past the commanded TARGET_ANGLE.
  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    390,  # deg
    # Vehicle model angle limits
    # Add extra tolerance for average banked road since safety doesn't have the roll
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^3

    # limit angle rate to both prevent a fault and for low speed comfort
    MAX_ANGLE_RATE=5,  # deg/20ms frame
  )

  # The column attenuates driver torque ~12x before the EPS measures it, so DRIVER_EPS_TORQUE
  # stays under 5 through deliberate wheel input and cannot be used to see an override.
  STEER_DRIVER_OVERRIDE = 18          # column torque for soft override, Nm
  STEER_DRIVER_DISENGAGE = 100        # column torque for hard disengage, Nm
  STEER_DRIVER_DISENGAGE_FRAMES = 5   # 100 ms of STEERING_TORQUE, mirrored in byd.h
  STEER_NOT_ACCEPTED_FRAMES = 10      # 200 ms of 50 Hz STEER_REQ
  WHEELSPEED_TO_KPH = 0.072           # 0.02 m/s/LSB, mirrored in byd.h
  EPB_DEBOUNCE_FRAMES = 8             # ~70 ms at 120 Hz; ignore 0x09-0x12 transitions

  # panda rate limits the first STEER_REQ=1 against our last 0x1E2, however old. After a
  # TX gap send REQ=0 at the measured angle first so that reference is fresh.
  STEER_WARMUP_FRAMES = 2             # 50 Hz frames
  # Re-engaging the EPS against a driver still on the wheel sets TORQUE_TEMP_FAILED;
  # a few within a second drop it to standby.
  STEER_YIELD_HOLD_SLOTS = 25         # 50 Hz frames, 0.5 s

  # EPS standby (LKS_PREPARED=1 with CRUISE_ACTIVATED=1, after >200 ms without 0x1E2)
  # ignores STEER_REQ until it sees the camera's REQ=0 / ACTIVE_LOW=0 ack. We send it.
  STEER_ACK_PERIOD = 10               # 50 Hz frames between ack pairs
  STEER_ACK_ATTEMPTS = 3              # then steerFaultTemporary until disengage

  # Camera LKS is a toggle on bus 2. Meaning A: off while we are enabled, on
  # again 2 s after we drop (panda HUD hold), off if our latch is off.
  # The camera checks the 0x3B0 counter: one frame mirroring the car's latest
  # 0x3B0 toggles it; anything out of sequence faults ACC within ~4 frames.
  LKS_PULSE_TICKS = 1                 # LKAS_ON_BTN frames per pulse
  LKS_PULSE_PERIOD = 5                # fallback slot if no fresh stock 0x3B0 arrives
  LKS_CONFIRM_FRAMES = 40             # 400 ms before one retry
  LKS_LOCKOUT_FRAMES = 50             # 500 ms after a real bus-0 press
  LKS_HUD_QUIET_FRAMES = 200          # 2 s, matches BYD_OP_HUD_TIMEOUT_US
  LKS_CAM_DEBOUNCE_FRAMES = 20        # 200 ms; treat 4 as still on
  LKS_EPS_IDLE_FRAMES = 20            # 200 ms PREP=1 CRUISE=0 before restore
  LKS_RECOVER_FRAMES = 200            # 2 s; retry want-off if a snap failed


class BydSafetyFlags(IntFlag):
  LKS_ON = 2


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


class CAR(Platforms):
  BYD_ATTO_3 = BydPlatformConfig(
    [BydCarDocs("BYD Atto 3 2022-25")],
    CarSpecs(wheelbase=2.72, steerRatio=14.8),
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
