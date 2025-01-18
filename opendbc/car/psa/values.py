from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import AngleRateLimit, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  STEER_STEP = 1  # spamming at 100 Hz works well, stock lkas is 20 Hz

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
  config: PSAPlatformConfig
  PSA_OPEL_CORSA_F = PSAPlatformConfig(
    [PSACarDocs("Opel Corsa F")],
    PSACarSpecs(mass=1530, wheelbase=2.540, steerRatio=17.6, centerToFrontRatio=0.44), # Peugeot e208
  )

# TODO: redo RX_offsets and extra_ecus. Negative offsets and subaddress >0xFF (0x100) were breaking the tests.
print("[DEBUG] Setting up PSA_RX_OFFSETS...")
PSA_RX_OFFSETS = {
  # Provide at least two offsets so we can see some queries,
  # and include multiple example ECUs in each list.
  0x20: (
    Ecu.fwdCamera,
    Ecu.engine,
    Ecu.epb,
    Ecu.transmission,
    Ecu.fwdRadar,
    Ecu.abs,
  ),
  0x100: (
    Ecu.combinationMeter,
    Ecu.gateway,
    Ecu.adas,
    Ecu.eps,
  ),
}
print(f"[DEBUG] PSA_RX_OFFSETS = {PSA_RX_OFFSETS}")

# We'll build up the requests list with debug prints. We'll run queries on bus=0 and bus=2.
requests_list = []
for rx_offset, whitelist_ecus in PSA_RX_OFFSETS.items():
  print(f"[DEBUG] Building firmware queries for rx_offset=0x{rx_offset:X}, ecus={whitelist_ecus}")
  for bus_num in (0, 2):
    print(f"[DEBUG]  -> Creating Request on bus={bus_num}")
    req = Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=whitelist_ecus,
      rx_offset=rx_offset,
      bus=bus_num,
      logging=True,
    )
    requests_list.append(req)

print("[DEBUG] Creating FwQueryConfig with the generated requests_list...")

FW_QUERY_CONFIG = FwQueryConfig(
  requests=requests_list,

  # If you want to forcibly detect some extra ECUs, put them here:
  extra_ecus=[
    # Example: these won't break anything, but might or might not respond
    (Ecu.eps, 0x75D, None),
    (Ecu.srs, 0x731, None),
  ],

  # Additional debug prints if you want them:
  # non_essential_ecus={ ... } could go here

)
print("[DEBUG] Finished constructing FW_QUERY_CONFIG.")

DBC = CAR.create_dbc_map()
