from dataclasses import dataclass, field
from enum import IntFlag
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, AngleRateLimit
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


@dataclass
class TeslaCarDocsHW3(CarDocs):
  # TODO: package not standard?
  package: str = "Traffic Aware Cruise Control"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_a]))


@dataclass
class TeslaCarDocsHW4(CarDocs):
  # TODO: package not standard?
  package: str = "Traffic Aware Cruise Control"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_b]))


@dataclass
class TeslaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.party: 'tesla_model3_party'})


class CAR(Platforms):
  TESLA_MODEL_3 = TeslaPlatformConfig(
    [
      # TODO: do we support 2017? It's HW3
      # TODO: do we support 2025? It's HW4
      TeslaCarDocsHW3("Tesla Model 3 2019-23"),
      TeslaCarDocsHW4("Tesla Model 3 2024"),
    ],
    CarSpecs(mass=1899., wheelbase=2.875, steerRatio=12.0),
  )
  TESLA_MODEL_Y = TeslaPlatformConfig(
    [
      TeslaCarDocsHW3("Tesla Model Y 2020-23"),
      TeslaCarDocsHW4("Tesla Model Y 2024"),
     ],
    CarSpecs(mass=2072., wheelbase=2.890, steerRatio=12.0),
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
  ]
)


class CANBUS:
  party = 0
  vehicle = 1
  autopilot_party = 2


GEAR_MAP = {
  "DI_GEAR_INVALID": CarState.GearShifter.unknown,
  "DI_GEAR_P": CarState.GearShifter.park,
  "DI_GEAR_R": CarState.GearShifter.reverse,
  "DI_GEAR_N": CarState.GearShifter.neutral,
  "DI_GEAR_D": CarState.GearShifter.drive,
  "DI_GEAR_SNA": CarState.GearShifter.unknown,
}


class CarControllerParams:
  # Angle rate limits are set using the Tesla Model Y VehicleModel such that they maximally meet ISO 11270
  # At 5 m/s, FSD has been seen hitting up to ~4 deg/frame with ~5 deg/frame at very low creeping speeds
  # At 30 m/s, FSD has been seen hitting mostly 0.1 deg/frame, sometimes 0.2 deg/frame, and rarely 0.3 deg/frame
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 25.], angle_v=[2.5, 1.5, 0.2])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 25.], angle_v=[5., 2.0, 0.3])
  STEER_STEP = 2  # Angle command is sent at 50 Hz
  ACCEL_MAX = 2.0    # m/s^2
  ACCEL_MIN = -3.48  # m/s^2
  JERK_LIMIT_MAX = 4.9  # m/s^3, ACC faults at 5.0
  JERK_LIMIT_MIN = -4.9  # m/s^3, ACC faults at 5.0


class TeslaSafetyFlags(IntFlag):
  LONG_CONTROL = 1


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 0.5
