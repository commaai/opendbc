from opendbc.car.fca_atlantis.fingerprints import FW_VERSIONS
from opendbc.car.fca_atlantis.values import CAR, ABS_ADDR, CAMERA_ADDR, EPS_ADDR, match_fw_to_car_fuzzy
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

EPS_2022 = b'\x01\x01S2FI04EB00\x00\x0b'
ABS_2022 = b'\x01\x01\x18\x06\x01\x00      \x00\x01'
CAM_2022 = b'\x01\x01\x06\x00\x06 \x00\x03 \x00\x00\x01\x00&'
EPS_NEW = b'\x01\x01FI07AA00-0\x00\x0b'
ABS_NEW = b'\x01\x01\x18\x08\x00\x00      \x00\x01'
CAM_NEW = b'\x01\x01\x06\x00\x07 \x00\x03 \x00\x00\x01\x00&'


def fuzzy(live):
  return match_fw_to_car_fuzzy(live, '', FW_VERSIONS)


def car_fw(versions):
  ecus = {EPS_ADDR: Ecu.eps, ABS_ADDR: Ecu.abs, CAMERA_ADDR: Ecu.fwdCamera}
  return [CarParams.CarFw(ecu=ecus[addr], fwVersion=v, address=addr[0], subAddress=0, brand='fca_atlantis')
          for addr, v in versions.items()]


class TestFcaAtlantis:
  def test_exact(self):
    exact, matches = match_fw_to_car(car_fw({EPS_ADDR: EPS_2022, ABS_ADDR: ABS_2022, CAMERA_ADDR: CAM_2022}), '')
    assert exact and matches == {CAR.RAM_PROMASTER}

  def test_exact_without_abs(self):
    exact, matches = match_fw_to_car(car_fw({EPS_ADDR: EPS_2022, CAMERA_ADDR: CAM_2022}), '', allow_fuzzy=False)
    assert exact and matches == {CAR.RAM_PROMASTER}

  def test_unseen_abs_falls_back_to_fuzzy(self):
    exact, matches = match_fw_to_car(car_fw({EPS_ADDR: EPS_2022, ABS_ADDR: ABS_NEW, CAMERA_ADDR: CAM_2022}), '')
    assert not exact and matches == {CAR.RAM_PROMASTER}

  def test_camera_required(self):
    # a van without the ADAS camera, or with an unknown camera, must not fingerprint
    for versions in ({EPS_ADDR: EPS_2022, ABS_ADDR: ABS_2022},
                     {EPS_ADDR: EPS_2022, ABS_ADDR: ABS_2022, CAMERA_ADDR: CAM_NEW}):
      _, matches = match_fw_to_car(car_fw(versions), '')
      assert matches == set(), versions

  def test_fuzzy_no_match(self):
    assert fuzzy({}) == set()
    assert fuzzy({EPS_ADDR: {EPS_2022}, ABS_ADDR: {ABS_2022}}) == set()  # camera required
    assert fuzzy({EPS_ADDR: {EPS_2022}, ABS_ADDR: {ABS_NEW}, CAMERA_ADDR: {CAM_NEW}}) == set()  # unknown camera
    assert fuzzy({EPS_ADDR: {EPS_NEW}, ABS_ADDR: {ABS_2022}, CAMERA_ADDR: {CAM_2022}}) == set()  # unknown EPS
    assert fuzzy({ABS_ADDR: {ABS_2022}, CAMERA_ADDR: {CAM_2022}}) == set()  # EPS required
    assert fuzzy({EPS_ADDR: {EPS_2022}, ABS_ADDR: {b'68288891AE'}, CAMERA_ADDR: {CAM_2022}}) == set()  # malformed ABS
