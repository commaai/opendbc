from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import AngleRateLimit, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1  # spamming at 100 Hz works well, stock lkas is ~20 Hz

  STEER_MAX = 390.0  # EPS can actuate the full range of steering
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 1.6, .3])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 7., 0.8])
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
  # Example single model; add more as needed
  PSA_OPEL_CORSA_F = PSAPlatformConfig(
    [PSACarDocs("Opel Corsa F")],
    PSACarSpecs(
      mass=1530,
      wheelbase=2.540,
      steerRatio=17.6,
      centerToFrontRatio=0.44,  # same ratio used for other small PSA platform cars
    ),
  )


PSA_ECU_ADDRS = [
  (Ecu.eps,              0x65D, None),  # was 0x75D + (-0x100)
  (Ecu.abs,              0x68D, None),  # was 0x6AD + (-0x20)
  (Ecu.fwdRadar,         0x665, None),  # 0x765 + (-0x100)
  (Ecu.fwdCamera,        0x644, None),
  (Ecu.engine,           0x65C, None),
  (Ecu.unknown,          0x660, None),
  (Ecu.transmission,     0x652, None),
  (Ecu.hybrid,           0x671, None),
  (Ecu.srs,              0x711, None),
  (Ecu.gateway,          0x66D, None),
  (Ecu.hud,              0x65F, None),
  (Ecu.combinationMeter, 0x64A, None),
  (Ecu.electricBrakeBooster, 0x710, None),
  (Ecu.shiftByWire,      0x695, None),
  (Ecu.adas,             0x642, None),
  (Ecu.cornerRadar,      0x688, None),
  (Ecu.hvac,             0x673, None),
  (Ecu.parkingAdas,      0x698, None),
  (Ecu.epb,              0x678, None),
  (Ecu.telematics,       0x664, None),
  (Ecu.body,             0x68F, None),
  (Ecu.dsu,              0x689, None),
  (Ecu.vsa,              0x646, None),
  (Ecu.programmedFuelInjection, 0x69D, None),
  (Ecu.debug,            0x604, None),
]

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # No negative offset
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
      logging=True,
    ),
  ],
  extra_ecus=PSA_ECU_ADDRS,
)


#
# Finally, create the DBC map so openpilot can find the correct .dbc files
#
DBC = CAR.create_dbc_map()
