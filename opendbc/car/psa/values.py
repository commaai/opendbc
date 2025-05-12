from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import AngleSteeringLimits, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  STEER_STEP = 1  # spamming at 100 Hz works well, stock lkas is ~20 Hz
  STEER_MAX = 390.0  # TODO: replace with ANGLE_LIMITS.  EPS can actuate the full range of steering

  # Angle rate limits are set to meet ISO 11270
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390, # deg
    ([0., 5., 25.], [2.5, 1.5, 0.2]),
    ([0., 5., 25.], [5., 2.0, 0.3]),
  )
  STEER_DRIVER_ALLOWANCE = 10  # Driver intervention threshold, 1.0 Nm
  EPS_MAX_TORQUE = 4.5 # TODO: max EPS torque seen in Nm
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
  })

class CAR(Platforms):
  PSA_OPEL_CORSA_F = PSAPlatformConfig(
    [PSACarDocs("Opel Corsa F")],
    PSACarSpecs(
      mass=1530,
      wheelbase=2.540,
      steerRatio=17.6,
      centerToFrontRatio=0.44,
    ),
  )

# Same as StdQueries.DEFAULT_DIAGNOSTIC_REQUEST
PSA_DIAGNOSTIC_REQUEST  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, 0x01])
# The Diagnostic response includes the ECU rx/tx timings
PSA_DIAGNOSTIC_RESPONSE = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x01])

PSA_SERIAL_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER,  0xF1, 0x8C])
PSA_SERIAL_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF1, 0x8C])

PSA_VERSION_REQUEST  = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER, 0xF0, 0xFE])
# TODO: Placeholder or info for uds module - The actual response is multi-frame TP
PSA_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF0, 0xFE])

PSA_RX_OFFSET = -0x20

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
        [PSA_DIAGNOSTIC_REQUEST],
        [PSA_DIAGNOSTIC_RESPONSE],
        rx_offset=PSA_RX_OFFSET,
        bus=1,
        logging=True,
        obd_multiplexing=False,
    ),
    Request(
        [PSA_SERIAL_REQUEST],
        [PSA_SERIAL_RESPONSE],
        rx_offset=PSA_RX_OFFSET,
        bus=1,
        logging=True,
        obd_multiplexing=False,
    ),
    Request(
        [PSA_VERSION_REQUEST],
        [PSA_VERSION_RESPONSE],
        rx_offset=PSA_RX_OFFSET,
        bus=1,
        logging=True,
        obd_multiplexing=False,
    ),
  ],
)

# TODO: multi-bus requests
# FW_QUERY_CONFIG = FwQueryConfig(
#   requests= [request for bus in (0, 1, 2) for request in [
#       Request(
#           [PSA_DIAGNOSTIC_REQUEST],
#           [PSA_DIAGNOSTIC_RESPONSE],
#           rx_offset=PSA_RX_OFFSET,
#           bus=bus,
#           logging=True,
#           obd_multiplexing=False,
#       ),
#       Request(
#           [PSA_VERSION_REQUEST],
#           [PSA_VERSION_RESPONSE],
#           rx_offset=PSA_RX_OFFSET,
#           bus=bus,
#           logging=True,
#           obd_multiplexing=False,
#       ),
#     ]
#   ]
# )

DBC = CAR.create_dbc_map()
