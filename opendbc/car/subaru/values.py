from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import ESSENTIAL_ECUS, FwQueryConfig, LiveFwVersions, OfflineFwVersions, Request, StdQueries, p16

Ecu = CarParams.Ecu


class CarControllerParams:
  def __init__(self, CP):
    self.STEER_STEP = 2                # how often we update the steer cmd
    self.STEER_DELTA_UP = 50           # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 70         # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 60   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 50  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1       # from dbc

    if CP.flags & SubaruFlags.GLOBAL_GEN2:
      # TODO: lower rate limits, this reaches min/max in 0.5s which negatively affects tuning
      self.STEER_MAX = 1000
      self.STEER_DELTA_UP = 40
      self.STEER_DELTA_DOWN = 40
    elif CP.carFingerprint == CAR.SUBARU_IMPREZA_2020:
      self.STEER_DELTA_UP = 35
      self.STEER_MAX = 1439
    else:
      self.STEER_MAX = 2047

  THROTTLE_MIN = 808
  THROTTLE_MAX = 3400

  THROTTLE_INACTIVE     = 1818   # corresponds to zero acceleration
  THROTTLE_ENGINE_BRAKE = 808    # while braking, eyesight sets throttle to this, probably for engine braking

  BRAKE_MIN = 0
  BRAKE_MAX = 600                # about -3.5m/s2 from testing

  RPM_MIN = 0
  RPM_MAX = 3600

  RPM_INACTIVE = 600             # a good base rpm for zero acceleration

  THROTTLE_LOOKUP_BP = [0, 2]
  THROTTLE_LOOKUP_V = [THROTTLE_INACTIVE, THROTTLE_MAX]

  RPM_LOOKUP_BP = [0, 2]
  RPM_LOOKUP_V = [RPM_INACTIVE, RPM_MAX]

  BRAKE_LOOKUP_BP = [-3.5, 0]
  BRAKE_LOOKUP_V = [BRAKE_MAX, BRAKE_MIN]


class SubaruSafetyFlags(IntFlag):
  GEN2 = 1
  LONG = 2
  PREGLOBAL_REVERSED_DRIVER_TORQUE = 4


class SubaruFlags(IntFlag):
  # Detected flags
  SEND_INFOTAINMENT = 1
  DISABLE_EYESIGHT = 2
  HAS_BSM = 128  # blind spot monitoring

  # Static flags
  GLOBAL_GEN2 = 4

  # Cars that temporarily fault when steering angle rate is greater than some threshold.
  # Appears to be all torque-based cars produced around 2019 - present
  STEER_RATE_LIMITED = 8
  PREGLOBAL = 16
  HYBRID = 32
  LKAS_ANGLE = 64


GLOBAL_ES_ADDR = 0x787
GEN2_ES_BUTTONS_DID = b'\x11\x30'


class CanBus:
  main = 0
  alt = 1
  camera = 2


class Footnote(Enum):
  GLOBAL = CarFootnote(
    "In the non-US market, openpilot requires the car to come equipped with EyeSight with Lane Keep Assistance.",
    Column.PACKAGE)
  EXP_LONG = CarFootnote(
    "Enabling longitudinal control (alpha) will disable all EyeSight functionality, including AEB, LDW, and RAB.",
    Column.LONGITUDINAL)


@dataclass
class SubaruCarDocs(CarDocs):
  package: str = "EyeSight Driver Assistance"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.subaru_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.GLOBAL])

  def init_make(self, CP: CarParams):
    if CP.alphaLongitudinalAvailable:
      self.footnotes.append(Footnote.EXP_LONG)


@dataclass
class SubaruPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'subaru_global_2017_generated'})

  def init(self):
    if self.flags & SubaruFlags.HYBRID:
      self.dbc_dict = {Bus.pt: 'subaru_global_2020_hybrid_generated'}


@dataclass
class SubaruGen2PlatformConfig(SubaruPlatformConfig):
  def init(self):
    super().init()
    self.flags |= SubaruFlags.GLOBAL_GEN2
    if not (self.flags & SubaruFlags.LKAS_ANGLE):
      self.flags |= SubaruFlags.STEER_RATE_LIMITED


class CAR(Platforms):
  # Global platform
  SUBARU_ASCENT = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Ascent 2019-21", "All")],
    CarSpecs(wheelbase=2.89, steerRatio=13.5),
  )
  SUBARU_OUTBACK = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Outback 2020-22", "All", car_parts=CarParts.common([CarHarness.subaru_b]))],
    CarSpecs(wheelbase=2.67, steerRatio=17),
  )
  SUBARU_LEGACY = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Legacy 2020-22", "All", car_parts=CarParts.common([CarHarness.subaru_b]))],
    SUBARU_OUTBACK.specs,
  )
  SUBARU_IMPREZA = SubaruPlatformConfig(
    [
      SubaruCarDocs("Subaru Impreza 2017-19"),
      SubaruCarDocs("Subaru Crosstrek 2018-19", video="https://youtu.be/Agww7oE1k-s?t=26"),
      SubaruCarDocs("Subaru XV 2018-19", video="https://youtu.be/Agww7oE1k-s?t=26"),
    ],
    CarSpecs(wheelbase=2.67, steerRatio=15),
  )
  SUBARU_IMPREZA_2020 = SubaruPlatformConfig(
    [
      SubaruCarDocs("Subaru Impreza 2020-22"),
      SubaruCarDocs("Subaru Crosstrek 2020-23"),
      SubaruCarDocs("Subaru XV 2020-21"),
    ],
    CarSpecs(wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.STEER_RATE_LIMITED,
  )
  # TODO: is there an XV and Impreza too?
  SUBARU_CROSSTREK_HYBRID = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Crosstrek Hybrid 2020", car_parts=CarParts.common([CarHarness.subaru_b]))],
    CarSpecs(wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.HYBRID,
  )
  SUBARU_FORESTER = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2019-21", "All")],
    CarSpecs(wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.STEER_RATE_LIMITED,
  )
  SUBARU_FORESTER_HYBRID = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester Hybrid 2020")],
    SUBARU_FORESTER.specs,
    flags=SubaruFlags.HYBRID,
  )
  # Pre-global
  SUBARU_FORESTER_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2017-18")],
    CarSpecs(wheelbase=2.67, steerRatio=20),
    {Bus.pt: 'subaru_forester_2017_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_LEGACY_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Legacy 2015-18")],
    CarSpecs(wheelbase=2.67, steerRatio=12.5),
    {Bus.pt: 'subaru_outback_2015_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_OUTBACK_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Outback 2015-17")],
    SUBARU_FORESTER_PREGLOBAL.specs,
    {Bus.pt: 'subaru_outback_2015_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_OUTBACK_PREGLOBAL_2018 = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Outback 2018-19")],
    SUBARU_FORESTER_PREGLOBAL.specs,
    {Bus.pt: 'subaru_outback_2019_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  # Angle LKAS
  SUBARU_FORESTER_2022 = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2022-24", "All", car_parts=CarParts.common([CarHarness.subaru_c]))],
    SUBARU_FORESTER.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )
  SUBARU_OUTBACK_2023 = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Outback 2023", "All", car_parts=CarParts.common([CarHarness.subaru_d]))],
    SUBARU_OUTBACK.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )
  SUBARU_ASCENT_2023 = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Ascent 2023", "All", car_parts=CarParts.common([CarHarness.subaru_d]))],
    SUBARU_ASCENT.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )


NON_ESSENTIAL_ECUS = {
  Ecu.eps: [c for c in CAR if c.config.flags & SubaruFlags.GLOBAL_GEN2],
}


def _get_fwd_camera_prefix(fw: bytes) -> tuple[int, bytes] | None:
  # EyeSight 0xf182 returns an 8- or 10-byte payload split into two equal halves.
  # Within ~10s of vehicle startup, the second half is zero-filled while the first
  # half already holds the primary EyeSight application version.
  if len(fw) not in (8, 10):
    return None
  half = len(fw) // 2
  if fw[:half] == b'\x00' * half:
    return None
  return (len(fw), fw[:half])


def _matches_fwd_camera(found_versions: set[bytes], expected_versions: list[bytes]) -> bool:
  if not found_versions:
    return False
  expected_set = set(expected_versions)
  expected_prefixes = {p for fw in expected_versions if (p := _get_fwd_camera_prefix(fw)) is not None}
  for fw in found_versions:
    if fw in expected_set:
      return True
    half = len(fw) // 2
    if len(fw) in (8, 10) and fw[half:] == b'\x00' * half and _get_fwd_camera_prefix(fw) in expected_prefixes:
      return True
  return False


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str, offline_fw_versions: OfflineFwVersions) -> set[str]:
  # Pass 1: exact match on all present essential non-camera ECUs + startup-prefix match on EyeSight fwdCamera.
  # Requiring fwdCamera (0x787) guarantees we never fingerprint a non-EyeSight vehicle lacking LKAS/ACC control.
  candidates: set[str] = set()
  for candidate, fws in offline_fw_versions.items():
    cam_matched = False
    valid = True
    for ecu, expected_versions in fws.items():
      ecu_type = ecu[0]
      addr = ecu[1:]
      found_versions = live_fw_versions.get(addr, set())

      if ecu_type == Ecu.fwdCamera:
        if not _matches_fwd_camera(found_versions, expected_versions):
          valid = False
          break
        cam_matched = True
        continue

      if not len(found_versions):
        if candidate in NON_ESSENTIAL_ECUS.get(ecu_type, []) or ecu_type not in ESSENTIAL_ECUS:
          continue
        valid = False
        break

      if not any(found_version in expected_versions for found_version in found_versions):
        valid = False
        break

    if valid and cam_matched:
      candidates.add(candidate)

  if len(candidates):
    return candidates

  # Pass 2: handle startup zero-suffixed fwdCamera when a powertrain ECU also received a minor revision update,
  # requiring fwdCamera prefix match + all essential ECUs present with matching format + >=1 exact non-camera ECU.
  non_cam_to_cars: dict[tuple[int, int | None, bytes], list[str]] = {}
  for candidate, fws in offline_fw_versions.items():
    for ecu, expected_versions in fws.items():
      if ecu[0] == Ecu.fwdCamera:
        continue
      for fw in expected_versions:
        non_cam_to_cars.setdefault((ecu[1], ecu[2], fw), []).append(candidate)

  for candidate, fws in offline_fw_versions.items():
    cam_matched = False
    exact_non_cam = 0
    valid = True
    for ecu, expected_versions in fws.items():
      ecu_type = ecu[0]
      addr = ecu[1:]
      found_versions = live_fw_versions.get(addr, set())

      if ecu_type == Ecu.fwdCamera:
        if not _matches_fwd_camera(found_versions, expected_versions):
          valid = False
          break
        cam_matched = True
        continue

      if not len(found_versions):
        if candidate in NON_ESSENTIAL_ECUS.get(ecu_type, []) or ecu_type not in ESSENTIAL_ECUS:
          continue
        valid = False
        break

      expected_lengths = {len(e) for e in expected_versions}
      if not any(len(v) in expected_lengths for v in found_versions):
        valid = False
        break

      if ecu_type == Ecu.abs:
        expected_prefixes = {e[:1] for e in expected_versions}
        if not any(v[:1] in expected_prefixes for v in found_versions):
          valid = False
          break

      for v in found_versions:
        unique_owners = non_cam_to_cars.get((addr[0], addr[1], v), [])
        if len(unique_owners) == 1 and unique_owners[0] != candidate:
          valid = False
          break
      if not valid:
        break

      if any(v in expected_versions for v in found_versions):
        exact_non_cam += 1

    if valid and cam_matched and exact_non_cam >= 1:
      candidates.add(candidate)

  return candidates


SUBARU_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
SUBARU_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)

# The EyeSight ECU takes 10s to respond to SUBARU_VERSION_REQUEST properly,
# log this alternate manufacturer-specific query
SUBARU_ALT_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf100)
SUBARU_ALT_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf100)

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"(?:[\x00-\xff]{4,5}|[\x00-\xff]{8}|[\x00-\xff]{10})",
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      logging=True,
    ),
    # Non-OBD requests
    # Some Eyesight modules fail on TESTER_PRESENT_REQUEST
    # TODO: check if this resolves the fingerprinting issue for the 2023 Ascent and other new Subaru cars
    Request(
      [SUBARU_VERSION_REQUEST],
      [SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
    ),
    Request(
      [SUBARU_ALT_VERSION_REQUEST],
      [SUBARU_ALT_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.DEFAULT_DIAGNOSTIC_REQUEST, StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE, StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      bus=0,
    ),
    # GEN2 powertrain bus query
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      bus=1,
      obd_multiplexing=False,
    ),
  ],
  # We don't get the EPS from non-OBD queries on GEN2 cars. Note that we still attempt to match when it exists
  non_essential_ecus=NON_ESSENTIAL_ECUS,
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

DBC = CAR.create_dbc_map()
