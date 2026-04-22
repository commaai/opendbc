import re
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, FSD_14_FW

Ecu = CarParams.Ecu

# Full Tesla EPS FW. Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# The only field whose meaning is evidence-backed is `platform` — its leading letter matches the
# car_model key in FW_VERSIONS across every current entry.
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  unknown_hw_digit=4  unknown_mid_letters=HP  unknown_series=015  major=05  minor=0
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),'
  rb'(?P<platform>[EYX])'
  rb'(?P<unknown_hw_digit>\d?)'
  rb'(?P<unknown_mid_letters>[A-Z]*)'
  rb'(?P<unknown_series>\d{3})'
  rb'\.(?P<major>\d+)'
  rb'(?:\.(?P<minor>\d+))?$'
)

# Only semantic claim we make from the version string: the leading letter identifies the car.
PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}

# FSD 14 FW: prefix-before-first-dot starts with the platform tag and ends with the version series,
# and the first component of the version is >= 4. Older series (E4H 014, Y4 002) are never FSD 14.
FSD_14_FW_RULE = {
  CAR.TESLA_MODEL_3: (b'E4H', b'015'),
  CAR.TESLA_MODEL_Y: (b'Y4',  b'003'),
}


class TestTeslaFingerprint:
  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)
        assert m is not None, f"Unparseable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_fsd_14_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      if car_model not in FSD_14_FW_RULE:
        continue
      tag, series = FSD_14_FW_RULE[car_model]
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        _, _, version = fw.partition(b',')
        is_fsd_14 = fw in FSD_14_FW.get(car_model, [])
        prefix, _, ver = version.partition(b'.')
        high_version = prefix.startswith(tag) and prefix.endswith(series) and int(ver.split(b'.')[0]) >= 4
        assert is_fsd_14 == high_version, f"{fw}"

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC
