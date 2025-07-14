from opendbc.car.rivian.fingerprints import FW_VERSIONS
from opendbc.car.rivian.values import CAR, FW_QUERY_CONFIG, WMI, ModelLine, ModelYear


class TestRivian:
  def test_custom_fuzzy_fingerprinting(self, subtests):
    for platform in CAR:
      with subtests.test(platform=platform.name):
        for wmi in WMI:
          for line in ModelLine:
            for year in ModelYear:
              for bad in (True, False):
                vin = ["0"] * 17
                vin[:3] = wmi
                vin[3] = line.value
                vin[9] = year.value
                if bad:
                  vin[3] = "Z"
                vin = "".join(vin)

                matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy({}, vin, FW_VERSIONS)
                # Gen1 years: N_2022, P_2023, R_2024, S_2025 (currently disabled)
                # Gen2 years: T_2026, U_2027, V_2028
                gen1_years = {ModelYear.N_2022, ModelYear.P_2023, ModelYear.R_2024}
                gen2_years = {ModelYear.T_2026, ModelYear.U_2027, ModelYear.V_2028}

                if platform == CAR.RIVIAN_R1_GEN1:
                  should_match = year in gen1_years and not bad
                elif platform == CAR.RIVIAN_R1_GEN2:
                  should_match = year in gen2_years and not bad
                else:
                  should_match = False

                assert (matches == {platform}) == should_match, f"Bad match for {platform} year {year}: expected {should_match}, got {matches}"
