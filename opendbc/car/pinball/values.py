from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs
from opendbc.car.fw_query_definitions import FwQueryConfig


class CAR(Platforms):
  COMMA_PINBALL = PlatformConfig(
    [CarDocs("comma pinball", package="All")],
    CarSpecs(mass=50, wheelbase=1.0, steerRatio=1.0),
    {Bus.main: 'comma_pinball'},
  )


FW_QUERY_CONFIG = FwQueryConfig(requests=[])

DBC = CAR.create_dbc_map()
