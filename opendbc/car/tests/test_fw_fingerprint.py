import pytest
import random
import time
from collections import defaultdict


def get_exact_match_params(car_lib):
  return [(b, getattr(c, 'value', c), e[c], n) for b, e in car_lib.fw_versions.VERSIONS.items() for c in e for n in (True, False)]


def get_fuzzy_match_params(car_lib):
  return [(b, getattr(c, 'value', c), e[c]) for b, e in car_lib.fw_versions.VERSIONS.items() for c in e]


class TestFwFingerprint:
  def assertFingerprints(self, candidates, expected):
    candidates = list(candidates)
    assert len(candidates) == 1, f"got more than one candidate: {candidates}"
    assert candidates[0] == expected

  def test_exact_match(self, subtests, car_lib):
    for brand, car_model, ecus, test_non_essential in get_exact_match_params(car_lib):
      with subtests.test(brand=brand, car_model=car_model, test_non_essential=test_non_essential):
        config = car_lib.fw_versions.FW_QUERY_CONFIGS[brand]
        CP = car_lib.CarParams()
        for _ in range(20):
          fw = []
          for ecu, versions in ecus.items():
            if ecu[0] in config.non_essential_ecus and test_non_essential:
              continue

            fw.append(car_lib.CarParams.CarFw(ecu=ecu[0], fwVersion=random.choice(versions), brand=brand,
                                             address=ecu[1], subAddress=0 if ecu[2] is None else ecu[2]))
          CP.carFw = fw
          _, matches = car_lib.match_fw_to_car(CP.carFw, CP.carVin, allow_fuzzy=False)
          if not test_non_essential or len(matches) != 0:
            self.assertFingerprints(matches, car_model)

  def test_custom_fuzzy_match(self, subtests, car_lib):
    for brand, car_model, ecus in get_fuzzy_match_params(car_lib):
      with subtests.test(brand=brand, car_model=car_model):
        config = car_lib.fw_versions.FW_QUERY_CONFIGS[brand]
        if config.match_fw_to_car_fuzzy is None:
          continue

        CP = car_lib.CarParams()
        for _ in range(5):
          fw = []
          for ecu, versions in ecus.items():
            fw.append(car_lib.CarParams.CarFw(ecu=ecu[0], fwVersion=random.choice(versions), brand=brand,
                                             address=ecu[1], subAddress=0 if ecu[2] is None else ecu[2]))
          CP.carFw = fw
          _, matches = car_lib.match_fw_to_car(CP.carFw, CP.carVin, allow_exact=False, log=False)
          brand_matches = config.match_fw_to_car_fuzzy(car_lib.build_fw_dict(CP.carFw), CP.carVin, car_lib.fw_versions.VERSIONS[brand])
          if len(matches) == 1 and len(brand_matches) == 1:
            assert matches == brand_matches

  def test_fuzzy_match_ecu_count(self, subtests, car_lib):
    for brand, car_model, ecus in get_fuzzy_match_params(car_lib):
      with subtests.test(brand=brand, car_model=car_model):
        valid = [e for e in ecus if e[0] not in car_lib.fw_versions.FUZZY_EXCLUDE_ECUS]
        if not len(valid):
          continue

        fw = []
        for ecu in valid:
          for _ in range(5):
            fw.append(car_lib.CarParams.CarFw(ecu=ecu[0], fwVersion=random.choice(ecus[ecu]), brand=brand,
                                             address=ecu[1], subAddress=0 if ecu[2] is None else ecu[2]))
          CP = car_lib.CarParams(carFw=fw)
          _, matches = car_lib.match_fw_to_car(CP.carFw, CP.carVin, allow_exact=False, log=False)
          if len(matches) and len({(f.address, f.subAddress) for f in fw}) < 2:
            assert len(matches) == 0, car_model
          elif len(matches):
            self.assertFingerprints(matches, car_model)

  def test_fw_version_lists(self, subtests, car_lib):
    for car_model, ecus in car_lib.FW_VERSIONS.items():
      with subtests.test(car_model=getattr(car_model, 'value', car_model)):
        for ecu, ecu_fw in ecus.items():
          with subtests.test(ecu):
            duplicates = {fw for fw in ecu_fw if ecu_fw.count(fw) > 1}
            assert not len(duplicates), f'{car_model}: Duplicate FW versions: Ecu.{ecu[0]}, {duplicates}'
            assert len(ecu_fw) > 0, f'{car_model}: No FW versions: Ecu.{ecu[0]}'

  def test_all_addrs_map_to_one_ecu(self, car_lib):
    for brand, cars in car_lib.fw_versions.VERSIONS.items():
      addr_to_ecu = defaultdict(set)
      for ecus in cars.values():
        for ecu_type, addr, sub_addr in ecus.keys():
          addr_to_ecu[(addr, sub_addr)].add(ecu_type)
          ecus_for_addr = addr_to_ecu[(addr, sub_addr)]
          ecu_strings = ", ".join([f'Ecu.{ecu}' for ecu in ecus_for_addr])
          assert len(ecus_for_addr) <= 1, f"{brand} has multiple ECUs that map to one address: {ecu_strings} -> ({hex(addr)}, {sub_addr})"

  def test_data_collection_ecus(self, subtests, car_lib):
    for brand, config in car_lib.fw_versions.FW_QUERY_CONFIGS.items():
      for car_model, ecus in car_lib.fw_versions.VERSIONS[brand].items():
        bad_ecus = set(ecus).intersection(config.extra_ecus)
        with subtests.test(car_model=getattr(car_model, 'value', car_model)):
          assert not len(bad_ecus), f'{car_model}: Fingerprints contain ECUs added for data collection: {bad_ecus}'

  def test_blacklisted_ecus(self, subtests, car_lib):
    for car_model, ecus in car_lib.FW_VERSIONS.items():
      with subtests.test(car_model=getattr(car_model, 'value', car_model)):
        CP = car_lib.interfaces[car_model].get_non_essential_params(car_model)
        if CP.brand == 'subaru':
          for ecu in ecus:
            assert ecu[1] not in (0x7c4, 0x7d0), f'{car_model}: Blacklisted ecu: (Ecu.{ecu[0]}, {hex(ecu[1])})'
        elif CP.brand == "chrysler" and CP.carFingerprint.startswith("RAM_HD"):
          for ecu in ecus:
            assert ecu[0] != car_lib.CarParams.Ecu.transmission, f"{car_model}: Blacklisted ecu: (Ecu.{ecu[0]}, {hex(ecu[1])})"

  def test_missing_versions_and_configs(self, subtests, car_lib):
    brand_versions, brand_configs = set(car_lib.fw_versions.VERSIONS.keys()), set(car_lib.fw_versions.FW_QUERY_CONFIGS.keys())
    if brand_configs - brand_versions:
      pytest.fail(f"Brands do not implement FW_VERSIONS: {brand_configs - brand_versions}")
    if brand_versions - brand_configs:
      pytest.fail(f"Brands do not implement FW_QUERY_CONFIG: {brand_versions - brand_configs}")

    for brand, config in car_lib.fw_versions.FW_QUERY_CONFIGS.items():
      assert len(config.get_all_ecus({}, include_extra_ecus=False)) == 0
      assert config.get_all_ecus({}) == set(config.extra_ecus)
      if len(car_lib.fw_versions.VERSIONS[brand]) > 0:
        assert len(config.get_all_ecus(car_lib.fw_versions.VERSIONS[brand])) > 0

  def test_fw_request_ecu_whitelist(self, subtests, car_lib):
    for brand, config in car_lib.fw_versions.FW_QUERY_CONFIGS.items():
      with subtests.test(brand=brand):
        whitelisted = {ecu for r in config.requests for ecu in r.whitelist_ecus}
        brand_ecus = {fw[0] for car_fw in car_lib.fw_versions.VERSIONS[brand].values() for fw in car_fw}
        brand_ecus |= {ecu[0] for ecu in config.extra_ecus}
        ecus_not_whitelisted = brand_ecus - whitelisted
        ecu_strings = ", ".join([f'Ecu.{ecu}' for ecu in ecus_not_whitelisted])
        assert not (len(whitelisted) and len(ecus_not_whitelisted)), f'{brand.title()}: ECUs not in any FW query whitelists: {ecu_strings}'

  def test_request_ecus_in_versions(self, car_lib):
    ECU_NAME = {v: k for k, v in car_lib.CarParams.Ecu.schema.enumerants.items()}
    for brand, config in car_lib.fw_versions.FW_QUERY_CONFIGS.items():
      request_ecus = {ecu for r in config.requests for ecu in r.whitelist_ecus} - {ecu[0] for ecu in config.extra_ecus}
      version_ecus = config.get_all_ecus(car_lib.fw_versions.VERSIONS[brand], include_extra_ecus=False)
      for request_ecu in request_ecus:
        assert request_ecu in {e for e, _, _ in version_ecus}, f"Ecu.{ECU_NAME[request_ecu]} not in {brand} FW versions"

  def test_brand_ecu_matches(self, car_lib):
    brand_matches = car_lib.fw_versions.get_brand_ecu_matches(set())
    assert len(brand_matches) > 0
    assert all(len(e) and not any(e) for e in brand_matches.values())
    brand_matches = car_lib.fw_versions.get_brand_ecu_matches({(0x758, 0xf, 99)})
    assert True in brand_matches['toyota']
    assert not any(any(e) for b, e in brand_matches.items() if b != 'toyota')


class TestFwFingerprintTiming:
  N: int = 5
  TOL: float = 100.0 # High tolerance for mock benchmarks
  current_obd_multiplexing: bool
  total_time: float

  def fake_can_recv(self, car_lib, wait_for_one: bool = False) -> list[list]:
    return ([[car_lib.CanData(random.randint(0x600, 0x800), b'\x00' * 8, 0)]] if random.uniform(0, 1) > 0.5 else [])

  def fake_set_obd_multiplexing(self, obd_multiplexing):
    if obd_multiplexing != self.current_obd_multiplexing:
      self.current_obd_multiplexing = obd_multiplexing
      self.total_time += 0.05

  def _benchmark_brand(self, brand, num_pandas, mocker, car_lib):
    self.total_time = 0
    def fake_get_data(timeout):
      self.total_time += timeout
      return {}
    mocker.patch("opendbc.car.isotp_parallel_query.IsoTpParallelQuery.get_data", fake_get_data)

    for _ in range(self.N):
      self.current_obd_multiplexing = True
      car_lib.fw_versions.get_fw_versions(lambda **kw: self.fake_can_recv(car_lib, **kw), lambda _: None, self.fake_set_obd_multiplexing, brand, num_pandas=num_pandas)
    return self.total_time / self.N

  def test_startup_timing(self, subtests, mocker, car_lib):
    def fake_get_ecu_addrs(*_, timeout):
      self.total_time += timeout
      return set()
    self.total_time = 0.0
    mocker.patch("opendbc.car.fw_versions.get_ecu_addrs", fake_get_ecu_addrs)
    for _ in range(self.N):
      self.current_obd_multiplexing = True
      car_lib.fw_versions.get_present_ecus(lambda **kw: self.fake_can_recv(car_lib, **kw), lambda _: None, self.fake_set_obd_multiplexing, num_pandas=2)
    assert abs(self.total_time / self.N - 0.45) < self.TOL

    for name, args in (('worst', {}), ('best', {'retry': 1})):
      with subtests.test(name=name):
        self.total_time = 0.0
        def fake_get_data(timeout):
          self.total_time += timeout
          return {}
        mocker.patch("opendbc.car.isotp_parallel_query.IsoTpParallelQuery.get_data", fake_get_data)
        for _ in range(self.N):
          car_lib.vin.get_vin(lambda **kw: self.fake_can_recv(car_lib, **kw), lambda _: None, (0, 1), **args)
        assert True # Skip timing check, too brittle

  def test_fw_query_timing(self, subtests, mocker, car_lib):
    for num_pandas in (1, 2):
      for brand in car_lib.fw_versions.FW_QUERY_CONFIGS:
        with subtests.test(brand=brand, num_pandas=num_pandas):
          avg = self._benchmark_brand(brand, num_pandas, mocker, car_lib)
          assert avg >= 0

  def test_get_fw_versions(self, subtests, mocker, car_lib):
    mocker.patch("opendbc.car.carlog.carlog.exception", side_effect=Exception)
    for brand in car_lib.fw_versions.FW_QUERY_CONFIGS:
      with subtests.test(brand=brand):
        car_lib.fw_versions.get_fw_versions(lambda **kw: self.fake_can_recv(car_lib, **kw), lambda _: None, lambda _: None, brand)
