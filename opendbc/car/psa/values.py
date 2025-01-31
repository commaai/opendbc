from dataclasses import dataclass, field

from cereal import car
from openpilot.selfdrive.car import AngleRateLimit, CarSpecs, dbc_dict, PlatformConfig, Platforms
from openpilot.selfdrive.car.docs_definitions import CarDocs, CarHarness, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 5  # LANE_KEEP_ASSIST, 20Hz

  STEER_MAX = 90.0  # Max angle for LKA
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 1.6, .3])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 7., 0.8])
  LKAS_MAX_TORQUE = 100  # TODO: verify (max seen is 60, signal max is 2047...)
  STEER_THRESHOLD = 25  # TODO: verify

  STEER_DRIVER_ALLOWANCE = 2  # Driver intervention threshold

  def __init__(self, CP):
    pass


@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


class CAR(Platforms):
  OPEL_CORSA_F = PlatformConfig(
    "OPEL CORSA F",
    [PSACarDocs("Vauxhall Corsa 2020")],
    CarSpecs(mass=1200, wheelbase=2.538, steerRatio=19.0),  # TODO: check steer ratio
    dbc_dict=dbc_dict('opel_corsa_f_hs1_generated', None, body_dbc='AEE2010_R3_HS2'),
  )

PSA_RX_OFFSETS = {
  -0x100: (Ecu.fwdCamera, Ecu.engine, Ecu.epb, Ecu.transmission, Ecu.hud, Ecu.vsa, Ecu.hvac, Ecu.hybrid, Ecu.unknown, Ecu.combinationMeter, Ecu.adas, Ecu.telematics, Ecu.gateway, Ecu.eps, Ecu.fwdRadar),
  -0x20: (Ecu.dsu, Ecu.abs, Ecu.parkingAdas, Ecu.electricBrakeBooster, Ecu.srs, Ecu.body, Ecu.programmedFuelInjection, Ecu.shiftByWire, Ecu.cornerRadar),
  -0xC0: (Ecu.debug),
}

FW_QUERY_CONFIG = FwQueryConfig(
  # TODO: find firmware requests
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=whitelist_ecus,
      rx_offset=rx_offset,
    ) for rx_offset, whitelist_ecus in PSA_RX_OFFSETS.items()
  ],
  extra_ecus=[
    # Note: these ecu names are wrong
    (Ecu.eps, 0x75D, -0x100),
    (Ecu.abs, 0x6AD, -0x20),
    (Ecu.fwdRadar, 0x765, -0x100),
    (Ecu.fwdCamera, 0x744, -0x100),
    (Ecu.engine, 0x75C, -0x100),
    (Ecu.unknown, 0x760, -0x100),
    (Ecu.transmission, 0x752, -0x100),
    (Ecu.hybrid, 0x771, -0x100),
    (Ecu.srs, 0x731, -0x20),
    (Ecu.gateway, 0x76D, -0x100),
    (Ecu.hud, 0x75F, -0x100),
    (Ecu.combinationMeter, 0x74A, -0x100),
    (Ecu.electricBrakeBooster, 0x730, -0x20),
    (Ecu.shiftByWire, 0x6B5, -0x20),
    (Ecu.adas, 0x742, -0x100),
    (Ecu.cornerRadar, 0x6A8, -0x20),
    (Ecu.hvac, 0x773, -0x100),
    (Ecu.parkingAdas, 0x6B8, -0x20),
    (Ecu.epb, 0x778, -0x100),
    (Ecu.telematics, 0x764, -0x100),
    (Ecu.body, 0x6AF, -0x20),
    (Ecu.dsu, 0x6A9, -0x20),
    (Ecu.vsa, 0x746, -0x100),
    (Ecu.programmedFuelInjection, 0x6BD, -0x20),
    (Ecu.debug, 0x6C4, -0xC0),
  ],
)

DBC = CAR.create_dbc_map()
