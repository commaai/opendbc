from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import CarSpecs, PlatformConfig, Platforms, dbc_dict
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness

Ecu = CarParams.Ecu


@dataclass
class PeroduaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.toyota_a]))

class PeroduaFlags(IntFlag):
  # Static flags
  ACC = 1

class CAR(Platforms):
  """
  For illustration, we create a PERODUA brand class with a single model:
  Myvi PSD. If you'd like to keep everything under one `CAR` class, you can do so.
  """
  MYVI_PSD = PlatformConfig(
    [PeroduaCarDocs("Perodua Myvi AV 2022+", "Perodua Smart Drive Assist")],
    # TODO: see if steer ratio correct, comma logs reports 13
    CarSpecs(mass=1025.,wheelbase=2.5,steerRatio=17.44,tireStiffnessFactor=0.9871,centerToFrontRatio=0.44),
    dbc_dict('perodua_psd_pt_generated', None),
    flags=PeroduaFlags.ACC
  )

class CarControllerParams:
  def __init__(self, CP):

    self.STEER_MAX = 300 # TODO: check if need change
    self.STEER_STEP = 3  # TODO: check if need change
    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV

    # for torque limit calculation
    self.STEER_DELTA_UP = 10
    self.STEER_DELTA_DOWN = 30

    self.STEER_REDUCE_FACTOR = 1000                 # how much to divide the steer when reducing fighting torque
    self.GAS_MAX = 2600                             # KommuActuator dac gas value
    self.GAS_STEP = 2                               # how often we update the longitudinal cmd
    self.BRAKE_ALERT_PERCENT = 60                   # percentage of brake to sound stock AEB alert
    self.ADAS_STEP = 5                              # 100/5 approx ASA frequency of 20 hz

DBC = CAR.create_dbc_map()

HUD_MULTIPLIER = 1.04

ACC_CAR = CAR.with_flags(PeroduaFlags.ACC)

def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions):
  # For now FW-based query not implemented
  return set()

# TODO: maybe fix this, i have no idea how this works
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.SHORT_TESTER_PRESENT_REQUEST, StdQueries.OBD_VERSION_REQUEST],
      [StdQueries.SHORT_TESTER_PRESENT_RESPONSE, StdQueries.OBD_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      bus=0,
    )],        # no active queries
  extra_ecus=[],      # or minimal list
)

