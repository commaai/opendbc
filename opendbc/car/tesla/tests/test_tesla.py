from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, FSD_14_FW

Ecu = CarParams.Ecu


class TestTeslaFingerprint:
  def test_fsd_14_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        _, _, version = fw.partition(b',')
        is_fsd_14 = fw in FSD_14_FW.get(car_model, [])

        if car_model == CAR.TESLA_MODEL_3:
          # Model 3: FSD 14 FW has 'P' in the Highland suffix (E4HP vs E4H)
          assert is_fsd_14 == version.startswith(b'E4HP'), f"{fw}"
        elif car_model == CAR.TESLA_MODEL_Y:
          # Model Y: FSD 14 FW is Y4x version >= 003.04
          prefix, _, ver = version.partition(b'.')
          y4_003 = prefix.startswith(b'Y4') and prefix.endswith(b'003')
          high_version = y4_003 and int(ver.split(b'.')[0]) >= 4
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
