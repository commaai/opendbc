import os
from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

SPEED_FROM_RPM = 0.008587

FIRMWARE_VERSION = "v0.3.1"
BIN_URL = f"https://github.com/commaai/body/releases/download/{FIRMWARE_VERSION}/body.bin.signed"
BIN_NAME = f"body-v1-{FIRMWARE_VERSION}.bin.signed"
BODY_DIR = os.path.dirname(os.path.realpath(__file__))
_PANDA_BIN_DIR = os.path.join(BODY_DIR, "../../../../panda/board/body/v1")
BIN_PATH = os.path.join(_PANDA_BIN_DIR, BIN_NAME) if os.path.isdir(_PANDA_BIN_DIR) else os.path.join(BODY_DIR, BIN_NAME)
FLASH_ADDR = 0x250
BUS = 0


class CarControllerParams:
  ANGLE_DELTA_BP = [0., 5., 15.]
  ANGLE_DELTA_V = [5., .8, .15]     # windup limit
  ANGLE_DELTA_VU = [5., 3.5, 0.4]   # unwind limit
  LKAS_MAX_TORQUE = 1               # A value of 1 is easy to overpower
  STEER_THRESHOLD = 1.0

  def __init__(self, CP):
    pass


class CAR(Platforms):
  COMMA_BODY = PlatformConfig(
    [CarDocs("comma body", package="All", video="https://youtu.be/VT-i3yRsX2s?t=2736")],
    CarSpecs(mass=9, wheelbase=0.406, steerRatio=0.5, centerToFrontRatio=0.44),
    {Bus.main: 'comma_body'},
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
