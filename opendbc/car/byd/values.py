from collections import namedtuple

from opendbc.car import DbcDict, PlatformConfig, Platforms, CarSpecs, structs, Bus
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness, SupportType
from dataclasses import dataclass, field

Ecu = CarParams.Ecu
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


class CarControllerParams:
    STEER_MAX = 300
    STEER_DELTA_UP = 17
    STEER_DELTA_DOWN = 17

    STEER_DRIVER_ALLOWANCE = 68
    STEER_DRIVER_MULTIPLIER = 3
    STEER_DRIVER_FACTOR = 1
    STEER_ERROR_MAX = 50
    # Steer torque clip = STEER_MAX - (DriverTorque - STEER_DRIVER_ALLOWANCE) * STEER_DRIVER_MULTIPLIER (Only work when DriverTorque > STEER_DRIVER_ALLOWANCE)
    # So DriverTorque(max) = STEER_MAX / STEER_DRIVER_MULTIPLIER + STEER_DRIVER_ALLOWANCE = 300/3+68 = 168
    # i.e. when drivertorque > 168, new_steer will be cliped to 0

    STEER_STEP = 2  # 2=50 Hz

    def __init__(self, CP):
        pass


@dataclass
class BYDCarDocs(CarDocs):
    package: str = "All"
    car_parts: CarParts = field(
        default_factory=CarParts.common([CarHarness.custom]))


@ dataclass
class BYDPlatformConfig(PlatformConfig):
    dbc_dict: DbcDict = field(
        default_factory=lambda: {Bus.pt: 'byd_general_pt'})


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


"""
    HAN_DM_20 = PlatformConfig(
        [CarDocs("BYD HAN DM 20", "All")],
        CarSpecs(mass=2080., wheelbase=2.920, steerRatio=15.0,
                 centerToFrontRatio=0.44, tireStiffnessFactor=0.81),
        dbc_dict('byd_han_dm_2020', None),
    )
    HAN_EV_20 = PlatformConfig(
        [CarDocs("BYD HAN EV 20", "All")],
        CarSpecs(mass=2100., wheelbase=2.959, steerRatio=15.0),
        dbc_dict('byd_han_dm_2020', None),
    )
"""

FW_QUERY_CONFIG = FwQueryConfig(
    requests=[
        Request(
            [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
            [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
            bus=0,
        ),
    ],
)


class CanBus:
    ESC = 0
    MRR = 1
    MPC = 2
    LOOPBACK = 128


BUTTONS = [
    Button(structs.CarState.ButtonEvent.Type.leftBlinker,
           "STALKS", "LeftIndicator", [0x01]),
    Button(structs.CarState.ButtonEvent.Type.rightBlinker,
           "STALKS", "RightIndicator", [0x01]),
    Button(structs.CarState.ButtonEvent.Type.accelCruise,
           "PCM_BUTTONS", "BTN_AccUpDown_Cmd", [0x02]),
    Button(structs.CarState.ButtonEvent.Type.decelCruise,
           "PCM_BUTTONS", "BTN_AccUpDown_Cmd", [0x03]),
    Button(structs.CarState.ButtonEvent.Type.cancel,
           "PCM_BUTTONS", "BTN_AccCancel", [0x01]),
]

DBC = CAR.create_dbc_map()
