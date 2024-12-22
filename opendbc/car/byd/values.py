from dataclasses import dataclass, field
from opendbc.car import CarSpecs, PlatformConfig, Platforms, dbc_dict
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

HUD_MULTIPLIER = 0.718

Ecu = CarParams.Ecu

# MIN_ACC_SPEED = 19. * CV.MPH_TO_MS
# PEDAL_TRANSITION = 10. * CV.MPH_TO_MS


class CarControllerParams:

    def __init__(self, CP):
        # maximum allow 150 degree per second, 100Hz loop means 1.5
        self.ANGLE_RATE_LIMIT_UP = 3
        self.ANGLE_RATE_LIMIT_DOWN = 3


@dataclass
class BYDCarDocs(CarDocs):
    package: str = "All"
    car_parts: CarParts = field(
        default_factory=CarParts.common([CarHarness.custom]))


@ dataclass
class BYDPlatformConfig(PlatformConfig):
    dbc_dict: dict = field(
        default_factory=lambda: dbc_dict('byd_general_pt', None))


class CAR(Platforms):
    BYD_ATTO3 = BYDPlatformConfig(
        [
            # The year has to be 4 digits followed by hyphen and 4 digits
            BYDCarDocs("BYD ATTO3 Electric 2022-24",
                       support_type=SupportType.COMMUNITY)
        ],
        CarSpecs(mass=1750, wheelbase=2.72, steerRatio=14.8,
                 tireStiffnessFactor=0.7983),
    )


# QZWF GR
FW_QUERY_CONFIG = FwQueryConfig(
    requests=[
        Request(
            [StdQueries.UDS_VERSION_REQUEST],
            [StdQueries.UDS_VERSION_RESPONSE],
            bus=0,
        ),
    ],
    extra_ecus=[
        # All known ECUs translated from the DBC file
        (Ecu.unknown, 0x1E2, None),
        (Ecu.unknown, 0x32D, None),  # ACC_HUD_ADAS
        (Ecu.unknown, 0x316, None),  # LKAS_HUD_ADAS
        (Ecu.unknown, 0x11F, None),  # STEER Angle
    ]
)

# So the DBC files contain Decimal Address. This needs to be converted to Hexadecimal Address. This is all that FW_QUERY_CONFIG Is looking for in most cases


DBC = CAR.create_dbc_map()
