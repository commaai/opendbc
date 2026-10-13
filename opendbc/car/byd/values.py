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
  # EPS standby needs the camera's REQ=0 / ACTIVE_LOW=0 ack frame
  EPS_ACK = 2
  # Camera LKS toggle on bus 2 and our 0x316 HUD hold
  LKS_HUD_CONTROL = 4


class CarControllerParams:
  # Mirrored in safety/modes/byd.h. Keep class-level: safety tests pass this
  # class itself as the limits object.
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

  ACCEL_MIN = -3.5                    # m/s^2
  ACCEL_MAX = 2.0                     # m/s^2

  # The column attenuates driver torque ~12x before the EPS measures it, so DRIVER_EPS_TORQUE
  # stays under 5 through deliberate wheel input and cannot be used to see an override.
  STEER_DRIVER_OVERRIDE = 22          # column torque for soft override, Nm
  STEER_DRIVER_DISENGAGE = 100        # column torque for hard disengage, Nm
  STEER_DRIVER_DISENGAGE_FRAMES = 5   # 100 ms of STEERING_TORQUE, mirrored in byd.h
  WHEELSPEED_TO_KPH = 0.072           # 0.02 m/s/LSB, mirrored in byd.h

  def __init__(self, CP):
    # Request low-pass and a tighter step below 30 kph. Breakpoints in m/s.
    self.LOW_SPEED_RATE_BP = [0., 2.78, 5.56, 8.33]   # 0, 10, 20, 30 kph
    self.LOW_SPEED_RATE_V = [2., 2., 3., 5.]          # deg/20ms frame
    self.LOW_SPEED_TAU_BP = [0., 2.78, 5.56, 8.33]    # 0, 10, 20, 30 kph
    self.LOW_SPEED_TAU_V = [0.3, 0.3, 0.15, 0.]       # s

    self.STEER_NOT_ACCEPTED_FRAMES = 10      # 200 ms of 50 Hz STEER_REQ
    self.EPB_DEBOUNCE_FRAMES = 8             # ~70 ms at 120 Hz; ignore 0x09-0x12 transitions

    # panda rate limits the first STEER_REQ=1 against our last 0x1E2, however old. After a
    # TX gap send REQ=0 at the measured angle first so that reference is fresh.
    self.STEER_WARMUP_FRAMES = 2             # 50 Hz frames
    # Re-engaging the EPS against a driver still on the wheel sets TORQUE_TEMP_FAILED;
    # a few within a second drop it to standby.
    self.STEER_YIELD_HOLD_SLOTS = 25         # 50 Hz frames, 0.5 s

    self.STOCK_AEB_ACCEL = -1.5              # m/s^2
    self.STANDSTILL_RESUME_CLEAR_SPEED = 2.0 # m/s; stock drops STANDSTILL_RESUME here, never set above

    if CP.flags & BydFlags.EPS_ACK:
      # EPS standby (LKS_PREPARED=1 with CRUISE_ACTIVATED=1, after >200 ms without 0x1E2)
      # ignores STEER_REQ until it sees the camera's REQ=0 / ACTIVE_LOW=0 ack. We send it.
      self.STEER_ACK_PERIOD = 10               # 50 Hz frames between ack pairs
      self.STEER_ACK_ATTEMPTS = 3              # then steerFaultTemporary and retries slow down
      self.STEER_ACK_RETRY_PERIOD = 50         # 50 Hz frames, 1 s between slow retries

    if CP.flags & BydFlags.LKS_HUD_CONTROL:
      # Camera LKS is a toggle on bus 2. Meaning A: off while we are enabled, on
      # again 2 s after we drop (panda HUD hold), off if our latch is off.
      # The camera checks the 0x3B0 counter: one frame mirroring the car's latest
      # 0x3B0 toggles it; anything out of sequence faults ACC within ~4 frames.
      self.LKS_PULSE_TICKS = 1                 # LKAS_ON_BTN frames per pulse
      self.LKS_PULSE_PERIOD = 5                # fallback slot if no fresh stock 0x3B0 arrives
      self.LKS_CONFIRM_FRAMES = 40             # 400 ms before one retry
      self.LKS_LOCKOUT_FRAMES = 50             # 500 ms after a real bus-0 press
      self.LKS_HUD_QUIET_FRAMES = 200          # 2 s, matches BYD_OP_HUD_TIMEOUT_US
      self.LKS_CAM_DEBOUNCE_FRAMES = 20        # 200 ms; treat 4 as still on
      self.LKS_EPS_IDLE_FRAMES = 20            # 200 ms PREP=1 CRUISE=0 before restore
      self.LKS_RECOVER_FRAMES = 200            # 2 s; retry want-off if a snap failed
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
