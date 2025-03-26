from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.values import CAR, FW_QUERY_CONFIG, PLATFORM_CODE_MAP, AP_VARIANT, VIN_YEAR_OFFSET
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from random import randint
Ecu = CarParams.Ecu

VARIANT_VERSIONS = {
  'DCS_UPDATE_0.0.0',
  'SingleECU_0.0.0',
  'Main_0.0.0',
  'Legacy3Y_0.0.0',
}

class TestTesla:

  def generate_random_version(self, parts=3):
    version_parts = []
    for _ in range(parts):
        version_parts.append(''.join(str(randint(1,10))))
    return '.'.join(version_parts)

  def generate_vin_year(self, year=2019):
    return f'5YJAAAAAA{chr(year - VIN_YEAR_OFFSET)}F000000'  # Tesla VIN year math

  def test_custom_fuzzy_fingerprinting(self, subtests):
    for platform in CAR:
      with subtests.test(platform=platform.name):
        for version in AP_VARIANT:
          for variant in VARIANT_VERSIONS:
            fwv = f'TeM{version}_{variant} ({randint(1,100)}),{PLATFORM_CODE_MAP[platform]}{self.generate_random_version()}'
            CP = CarParams(
              carFw=[
                CarParams.CarFw(
                  ecu=Ecu.eps,
                  fwVersion=str.encode(fwv),
                  address=0x730,
                  subAddress=0
                ),
              ])
            vin = self.generate_vin_year(2019 + randint(0, 4))
            matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), vin, FW_VERSIONS)
            assert matches == {platform}