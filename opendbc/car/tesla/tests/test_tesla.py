import re

from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, FSD_14_FW

Ecu = CarParams.Ecu

# Parses EPS version after comma: model letter (E/Y/X), optional '4' for HW4, variant letters, version series, minor version
# e.g. E4HP015.04.5 -> model=E, hw4=4, variant=HP, series=015, minor=04
FW_PATTERN = re.compile(rb',(?P<model>[EYX])(?P<hw4>4?)(?P<variant>[A-Z]*)(?P<series>\d{3})\.(?P<minor>\d+)')


class TestTeslaFingerprint:
  def test_fsd_14_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        match = FW_PATTERN.search(fw)
        assert match, f"Unparseable FW: {fw}"
        is_fsd_14 = fw in FSD_14_FW.get(car_model, [])

        if car_model == CAR.TESLA_MODEL_3:
          # Model 3: FSD 14 FW has 'P' in the Highland variant (E4HP vs E4H)
          assert is_fsd_14 == (b'P' in match.group('variant')), f"{fw}"
        elif car_model == CAR.TESLA_MODEL_Y:
          # Model Y: FSD 14 is HW4 (Y4), version series 003 with minor >= 4 (002 is never FSD 14)
          assert is_fsd_14 == (match.group('series') == b'003' and int(match.group('minor')) >= 4), f"{fw}"

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
