from dataclasses import dataclass, field
from enum import IntFlag
from opendbc.car import Bus, DbcDict, PlatformConfig, Platforms, CarSpecs
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  STEER_MAX = 300
  STEER_DELTA_UP = 7
  STEER_DELTA_DOWN = 10

  STEER_DRIVER_ALLOWANCE = 68
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50

  STEER_STEP = 2  #100/2=50hz
  STEER_SOFTSTART_STEP = 6 # 20ms(50Hz) * 300 / 6 = 1000ms. This means the clip ceiling will be increased to 300 in 1000ms

  ACC_STEP = 2    #50hz

  ACCEL_MAX = 2.0
  ACCEL_MIN = -3.5

  K_DASHSPEED = 0.0719088 #convert pulse to kph

  USE_STEERING_SPEED_LIMITER = False

  # op long control
  K_accel_jerk_upper = 0.1
  K_accel_jerk_lower = 0.5
  K_jerk_xp =            [   4,   10,   20,   40,   80]  # meters
  K_jerk_base_lower_fp = [-2.3, -1.8, -1.4, -1.0, -0.4]
  K_jerk_base_upper_fp = [ 0.8,  0.7,  0.6,  0.3,  0.2]

  def __init__(self, CP):
    pass

#FD to be added later
class BydSafetyFlags(IntFlag):
  HAN_TANG_DMEV = 0x1 #pre 2021 models with veoneer mpc/radar solution
  TANG_DMI = 0x2 #note tang dmi is not tang dm
  SONG_PLUS_DMI = 0x4 #note song pro is similar but not song dmi
  QIN_PLUS_DMI = 0x8
  YUAN_PLUS_DMI_ATTO3 = 0x10 #yuan plus is atto3


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  #todo add docs and harness info

@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "byd_han_dmev_2020"})
  #todo add dbc for other models

class CAR(Platforms):
  BYD_HAN_DM_20 = BydPlatformConfig(
    [BydCarDocs("BYD HAN DM 20")],
    CarSpecs(mass=2080., wheelbase=2.920, steerRatio=16.8, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )
  BYD_HAN_EV_20 = BydPlatformConfig(
    [BydCarDocs("BYD HAN EV 20")],
    CarSpecs(mass=2100., wheelbase=2.959, steerRatio=16.8, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  #The following parameters are likely be incorrect, developers please fill and fix them.

  BYD_TANG_DM = BydPlatformConfig(
    [BydCarDocs("BYD TANG DM")],
    CarSpecs(mass=2250., wheelbase=2.820, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_TANG_DMI_21 = BydPlatformConfig(
    [BydCarDocs("BYD TANG DMI 21")],
    CarSpecs(mass=2153., wheelbase=2.820, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_SONG_PLUS_DMI_21 = BydPlatformConfig(
    [BydCarDocs("BYD SONG PLUS DMI 21")],
    CarSpecs(mass=1785., wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_SONG_PLUS_DMI_22 = BydPlatformConfig(
    [BydCarDocs("BYD SONG PLUS DMI 22")],
    CarSpecs(mass=1785., wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_SONG_PLUS_DMI_23 = BydPlatformConfig(
    [BydCarDocs("BYD SONG PLUS DMI 23")],
    CarSpecs(mass=1785., wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_SONG_PRO_DMI_22 = BydPlatformConfig(
    [BydCarDocs("BYD SONG PRO DMI 22")],
    CarSpecs(mass=1670., wheelbase=2.712, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_QIN_PLUS_DMI_23 = BydPlatformConfig(
    [BydCarDocs("BYD QIN PLUS DMI 23")],
    CarSpecs(mass=1580., wheelbase=2.718, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_YUAN_PLUS_DMI_22 = BydPlatformConfig(
    [BydCarDocs("BYD YUAN PLUS DMI 22")],
    CarSpecs(mass=1625., wheelbase=2.720, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )


class LKASConfig:
  DISABLE = 0
  ALARM = 1
  LKA = 2
  ALARM_AND_LKA = 3

class CanBus:
  ESC = 0
  MRR = 1
  MPC = 2

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=CanBus.ESC,
    ),
  ],
)

PLATFORM_HANTANG_DMEV = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}
PLATFORM_TANG_DMI = {CAR.BYD_TANG_DMI_21}
PLATFORM_SONG_PLUS_DMI = {CAR.BYD_SONG_PLUS_DMI_21, CAR.BYD_SONG_PLUS_DMI_22, CAR.BYD_SONG_PLUS_DMI_23, CAR.BYD_SONG_PRO_DMI_22}
PLATFORM_QIN_PLUS_DMI = {CAR.BYD_QIN_PLUS_DMI_23}
PLATFORM_YUAN_PLUS_DMI_ATTO3 = {CAR.BYD_YUAN_PLUS_DMI_22}

# power train canbus is located and accessible in in MPC connector
MPC_ACC_CAR = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}

# power train canbus contains mrr radar info
PT_RADAR_CAR = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}

# use torque lat control, otherwise use angle mode
TORQUE_LAT_CAR = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}

# use experimental long mode
EXP_LONG_CAR = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}

DBC = CAR.create_dbc_map()
