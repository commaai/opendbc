from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import AngleRateLimit, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
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


################################################################################
# Example: Common PSA ECU Address Pairs
#
# Adjust, remove, or add pairs from the official PSA diag ID list. Each entry is:
#   Ecu.<enum_name>: (request_id, response_id)
################################################################################

# PSA Diagnostic Route:
# 6a7075a4fdd765ee/0000024b--450fa192db

PSA_ECU_ADDRS = {
  # Engine ECU (example: 0x6A8 -> 0x688 offset = -0x20, sometimes it's 0xE0 difference)
  Ecu.engine:        (0x6A8, 0x688), #(no reply)
  # Transmission ECU
  Ecu.transmission:  (0x6A9, 0x689), #(no reply)
  # VCU (Electric Vehicle Control Unit) e.g. 0x6A2 -> 0x682 offset=+0x40
  Ecu.hybrid:        (0x6A2, 0x682),
  # OBC / DC-DC converter, e.g. 0x590 -> 0x58F offset=-1
  Ecu.electricBrakeBooster: (0x590, 0x58F),
  # BSI / Body
  Ecu.body:          (0x752, 0x652), #(no reply)
  # Instrument Cluster
  Ecu.combinationMeter: (0x75F, 0x65F), #(no reply)
  # Telematic / Headunit
  Ecu.telematics:    (0x764, 0x664), #(no reply)
  # Steering / DIRECTN
  Ecu.eps:           (0x6B5, 0x695),
  # ABS/ESP
  Ecu.abs:           (0x6AD, 0x68D),  #(no reply)
  # HUD or Display
  Ecu.hud:           (0x765, 0x665), #(no reply)

  # CVM / CVM_3, CVM_2, CVM
  Ecu.gateway:       (0x74A, 0x64A),
  # HCU2 / Hydraulic Control Unit
  Ecu.hvac:       (0x6A6, 0x686),
  # MSB / TBMU, TBMU_PHEV, BMU_CTE1
  Ecu.srs:      (0x6B4, 0x694),
  # ARTIV / ARTIV, RADAR_AV_4, LIDAR, ARTIV_UDS
  Ecu.epb:      (0x6B6, 0x696),
  # BAAST
  Ecu.vsa:      (0x732, 0x712),
  # CTPA
  Ecu.debug:      (0x737, 0x717),
}

################################################################################
# Building the openpilot FwQueryConfig
#
# For each ECU, we replicate what your official diag logs do:
#    1) 0x10 0x03 -> 0x50 0x03  (Extended Diag Session)
#    2) 0x22 F0 FE -> 0x62 F0 FE  (Read Data ID 0xF0FE)
#    3) 0x10 0x01 -> 0x50 0x01  (Return to Default Session)
#
# The final “0x62 F0 FE ...” data is treated as the firmware version.
#
# If your target ECU does *not* respond to 0x22 F0 FE, you may need to change
# that ID (for example 0xF1A0, 0xF184, etc.) to match the actual PSA ID for
# SW version on that particular ECU.
################################################################################

def build_psa_requests() -> list[Request]:
  requests = []
  for ecu_name, (req_id, resp_id) in PSA_ECU_ADDRS.items():
    # Calculate offset for openpilot.  Example: if requests go out at 0x590
    # and responses come back at 0x58F, then offset = 0x58F - 0x590 = -1
    rx_offset = resp_id - req_id

    requests.append(
      Request(
        request=[
          b"\x10\x03",       # extended diag session
          b"\x22\xF0\xFE",   # read SW version from ID F0FE
          b"\x10\x01",       # back to default session
        ],
        response=[
          b"\x50\x03",       # DSC response, extended session
          b"\x62\xF0\xFE",   # positive response to ReadDataByID
          b"\x50\x01",       # DSC response, default session
        ],
        whitelist_ecus=[ecu_name],  # label so the parser knows which Ecu
        bus=0,                      # PSA typically on CAN bus 0
        rx_offset=rx_offset,
      )
    )
  return requests

FW_QUERY_CONFIG = FwQueryConfig(
  requests=build_psa_requests(),
  # If you have any ecus you do *not* want to treat as essential for matching,
  # you can add them below. Key = Ecu, Value = list of Car model strings
  non_essential_ecus={},
  # Extra ecus you only want to log but not use in fingerprint
  extra_ecus=[],
)

DBC = CAR.create_dbc_map()

# TODO: Remove
#   extra_ecus=[
#     # TODO: Negative offsets and subaddress >0xFF (0x100) were breaking the tests.
#     # Note: these ecu names are wrong
#     # (Ecu.eps, 0x75D, -0x100),
#     # (Ecu.abs, 0x6AD, -0x20),
#     # (Ecu.fwdRadar, 0x765, -0x100),
#     # (Ecu.fwdCamera, 0x744, -0x100),
#     # (Ecu.engine, 0x75C, -0x100),
#     # (Ecu.unknown, 0x760, -0x100),
#     # (Ecu.transmission, 0x752, -0x100),
#     # (Ecu.hybrid, 0x771, -0x100),
#     # (Ecu.srs, 0x731, -0x20),
#     # (Ecu.gateway, 0x76D, -0x100),
#     # (Ecu.hud, 0x75F, -0x100),
#     # (Ecu.combinationMeter, 0x74A, -0x100),
#     # (Ecu.electricBrakeBooster, 0x730, -0x20),
#     # (Ecu.shiftByWire, 0x6B5, -0x20),
#     # (Ecu.adas, 0x742, -0x100),
#     # (Ecu.cornerRadar, 0x6A8, -0x20),
#     # (Ecu.hvac, 0x773, -0x100),
#     # (Ecu.parkingAdas, 0x6B8, -0x20),
#     # (Ecu.epb, 0x778, -0x100),
#     # (Ecu.telematics, 0x764, -0x100),
#     # (Ecu.body, 0x6AF, -0x20),
#     # (Ecu.dsu, 0x6A9, -0x20),
#     # (Ecu.vsa, 0x746, -0x100),
#     # (Ecu.programmedFuelInjection, 0x6BD, -0x20),
#     # (Ecu.debug, 0x6C4, -0xC0),
#   ],
# )
