from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import AngleRateLimit, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  STEER_STEP = 1  # spamming at 100 Hz works well, stock lkas is 20 Hz

  STEER_MAX = 390.0  # EPS can actuate the full range of steering
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[5., .8, .15])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[5., 3.5, 0.4])
  STEER_DRIVER_ALLOWANCE = 10  # Driver intervention threshold, 1 Nm

  def __init__(self, CP):
    pass

@dataclass(frozen=True, kw_only=True)
class PSACarSpecs(CarSpecs):
  tireStiffnessFactor: float = 1.03

@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))

@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'AEE2010_R3',
    Bus.adas: 'AEE2010_R3',
    Bus.main: 'AEE2010_R3',
  })

class CAR(Platforms):
  config: PSAPlatformConfig
  PSA_OPEL_CORSA_F = PSAPlatformConfig(
    [PSACarDocs("Opel Corsa F")],
    PSACarSpecs(mass=1530, wheelbase=2.540, steerRatio=17.6, centerToFrontRatio=0.44), # Peugeot e208
  )

PSA_RX_OFFSETS = {
  0x100: (
    Ecu.fwdCamera, Ecu.engine, Ecu.epb, Ecu.transmission, Ecu.hud,
    Ecu.vsa, Ecu.hvac, Ecu.hybrid, Ecu.unknown, Ecu.combinationMeter,
    Ecu.adas, Ecu.telematics, Ecu.gateway, Ecu.eps, Ecu.fwdRadar
  ),
  0x20: (
    Ecu.dsu, Ecu.abs, Ecu.parkingAdas, Ecu.electricBrakeBooster,
    Ecu.srs, Ecu.body, Ecu.programmedFuelInjection, Ecu.shiftByWire,
    Ecu.cornerRadar
  ),
  0xC0: (
    Ecu.debug,
  ),
}

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=whitelist_ecus,
      rx_offset=rx_offset,
    ) for rx_offset, whitelist_ecus in PSA_RX_OFFSETS.items()
  ],
  extra_ecus=[],
)

DBC = CAR.create_dbc_map()
