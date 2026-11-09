from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.fw_query_definitions import FwQueryConfig, Request
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 2  # ADAS_LKA (0x83) is sent at 50 Hz

  # Must match opendbc/safety/modes/leapmotor.h. openpilot's standard vehicle-model limits: lateral accel and
  # lateral jerk of about 3.6 (m/s^2, m/s^3), so near-full lock is allowed up to about 15 km/h (U-turns) and
  # about 50 deg at 50 km/h. Stock LCC stays below 2.3 m/s^2 on every recorded route.
  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    500,  # deg; mechanical lock is about 522 deg, keep a margin so the EPS never drives into the end stop
    MAX_ANGLE_RATE=5,  # deg per 20 ms frame (250 deg/s): lock to lock in about 4 s. EPS fault rate untested
  )
  # Driver torque from EPS_STEERING.STEER_TORQUE_DRIVER (unit unknown). Hands-off LCC never exceeds 3 on the
  # filmed route and 5 over 36 min of LCC; holding full lock parked reaches 21-25. With the 5-frame debounce in
  # carstate, 4 gave zero steeringPressed events over those 36 min. Steering against openpilot will need more
  # torque than free manual steering, so detection then should be higher than the 22% seen in manual turns.
  STEER_DRIVER_ALLOWANCE = 4


@dataclass
class LeapmotorCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Centering Control (LCC)"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class LeapmotorPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'leapmotor_b10',
  })


class CAR(Platforms):
  # wheelbase: Italian brochure, B10 Life with the 56.2 kWh Pro battery.
  # steerRatio 15.2 is measured from slow figure-eights (steering angle vs. yaw rate / speed).
  LEAPMOTOR_B10 = LeapmotorPlatformConfig(
    [LeapmotorCarDocs("Leapmotor B10 2025")],
    CarSpecs(wheelbase=2.735, steerRatio=15.2),
  )


# Part number (F187), physical request = reply minus 8: EPS 0x735, brake 0x736, front camera 0x756, all confirmed by a
# read-only UDS probe on the car. No tester present first: the camera did not answer openpilot's query when it led with
# one, but answers F187 directly. At ignition, before READY, only the brake answers (the EPS stays silent until READY,
# the camera is slow), so the EPS and camera are non-essential; their part numbers are still checked when present.
PART_NUMBER_REQUEST = bytes([0x22, 0xF1, 0x87])
PART_NUMBER_RESPONSE = bytes([0x62, 0xF1, 0x87])

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [PART_NUMBER_REQUEST],
      [PART_NUMBER_RESPONSE],
      whitelist_ecus=[Ecu.eps, Ecu.abs, Ecu.fwdCamera],
      bus=0,
    ),
  ],
  non_essential_ecus={
    Ecu.eps: [CAR.LEAPMOTOR_B10],
    Ecu.fwdCamera: [CAR.LEAPMOTOR_B10],
  },
  # part numbers such as 3401100-AG01, zero-padded to 19 bytes
  fw_version_regex=br"\d{7}-[A-Z]{2}\d{2}\x00{7}",
)

DBC = CAR.create_dbc_map()
