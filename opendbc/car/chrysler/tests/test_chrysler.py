import pytest

from opendbc.car.chrysler.fingerprints import FW_VERSIONS
from opendbc.car.chrysler.values import PLATFORM_CODE_ECUS, match_fw_to_car_fuzzy
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.structs import CarParams

CarFw = CarParams.CarFw


def _mutate_revision(fw: bytes) -> bytes:
  clean_fw = fw.rstrip(b"\x00").strip()
  return clean_fw[:-2] + b"ZZ"


@pytest.mark.parametrize("car_model, ecus", FW_VERSIONS.items())
def test_platform_code_ecus_available(car_model, ecus):
  platform_ecus = [ecu for ecu in ecus if ecu[0] in PLATFORM_CODE_ECUS]
  # Require at least two ECUs with platform part numbers so fuzzy matching stays reliable
  assert len(platform_ecus) >= 2, f"{car_model}: missing platform ECUs for fuzzy fingerprinting"


@pytest.mark.parametrize("car_model, ecus", FW_VERSIONS.items())
def test_revision_tolerant_fuzzy(car_model, ecus):
  # Swap every platform ECU's revision with a new value to mimic unseen firmware
  fw = []
  for (ecu_name, addr, sub_addr), versions in ecus.items():
    if ecu_name not in PLATFORM_CODE_ECUS:
      continue

    mutated_fw = _mutate_revision(versions[0])
    fw.append(CarFw(ecu=ecu_name, fwVersion=mutated_fw, brand="chrysler",
                    address=addr, subAddress=0 if sub_addr is None else sub_addr))

  CP = CarParams(carFw=fw)
  live_fw = build_fw_dict(CP.carFw, filter_brand="chrysler")
  matches = match_fw_to_car_fuzzy(live_fw, CP.carVin, FW_VERSIONS)

  assert car_model in matches
