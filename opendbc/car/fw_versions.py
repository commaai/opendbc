from collections import defaultdict
from collections.abc import Callable, Iterator
from typing import Protocol, TypeVar, Any

from opendbc.car import uds
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.structs import CarParams
from opendbc.car.ecu_addrs import get_ecu_addrs
from opendbc.car.fw_query_definitions import ESSENTIAL_ECUS, AddrType, EcuAddrBusType, FwQueryConfig, LiveFwVersions, OfflineFwVersions
from opendbc.car_discovery import get_interface_attr

Ecu = CarParams.Ecu
FUZZY_EXCLUDE_ECUS = [Ecu.fwdCamera, Ecu.fwdRadar, Ecu.eps, Ecu.debug]

T = TypeVar('T')

def chunks(l: list[T], n: int = 128) -> Iterator[list[T]]:
  for i in range(0, len(l), n):
    yield l[i:i + n]

def is_brand(brand: str, filter_brand: str | None) -> bool:
  return filter_brand is None or brand == filter_brand

def __getattr__(name):
  if name == 'VERSIONS':
    val = get_interface_attr('FW_VERSIONS', ignore_none=True)
  elif name == 'FW_QUERY_CONFIGS':
    val = get_interface_attr('FW_QUERY_CONFIG', ignore_none=True)
  elif name == 'MODEL_TO_BRAND':
    val = {c: b for b, e in VERSIONS.items() for c in e}
  elif name == 'REQUESTS':
    val = [(brand, config, r) for brand, config in FW_QUERY_CONFIGS.items() for r in config.requests]
  else:
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
  globals()[name] = val
  return val

def __dir__():
  return sorted(list(globals().keys()) + ['FW_QUERY_CONFIGS', 'VERSIONS', 'MODEL_TO_BRAND', 'REQUESTS'])


T = TypeVar('T')
ObdCallback = Callable[[bool], None]


def chunks(l: list[T], n: int = 128) -> Iterator[list[T]]:
  for i in range(0, len(l), n):
    yield l[i:i + n]


def is_brand(brand: str, filter_brand: str | None) -> bool:
  """Returns if brand matches filter_brand or no brand filter is specified"""
  return filter_brand is None or brand == filter_brand


def build_fw_dict(fw_versions: list[CarParams.CarFw], filter_brand: str | None = None) -> dict[AddrType, set[bytes]]:
  fw_versions_dict: defaultdict[AddrType, set[bytes]] = defaultdict(set)
  for fw in fw_versions:
    if is_brand(fw.brand, filter_brand) and not fw.logging:
      sub_addr = fw.subAddress if fw.subAddress != 0 else None
      fw_versions_dict[(fw.address, sub_addr)].add(bytes(fw.fwVersion))
  return dict(fw_versions_dict)


class MatchFwToCar(Protocol):
  def __call__(self, live_fw_versions: LiveFwVersions, match_brand: str | None = None, log: bool = True) -> set[str]:
    ...


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, match_brand: str | None = None, log: bool = True, exclude: str | None = None) -> set[str]:
  from opendbc.car.fingerprints import FW_VERSIONS

  # Build lookup table (addr, sub_addr, fw) -> list of candidate cars
  all_fw_versions = defaultdict(list)
def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, match_brand: str | None = None, log: bool = True, exclude: str | None = None) -> set[str]:
  from opendbc.car.fingerprints import FW_VERSIONS
  from opendbc.car.fw_versions import MODEL_TO_BRAND

  # Build lookup table (addr, sub_addr, fw) -> list of candidate cars
  all_fw_versions = defaultdict(list)
  for candidate, fw_by_addr in FW_VERSIONS.items():
    if not is_brand(MODEL_TO_BRAND[candidate], match_brand):
      continue
    if candidate == exclude:
      continue
    for addr, fws in fw_by_addr.items():
      if addr[0] in FUZZY_EXCLUDE_ECUS:
        continue
      for f in fws:
        all_fw_versions[(addr[1], addr[2], f)].append(candidate)

  matched_ecus = set()
  match: str | None = None
  for addr, versions in live_fw_versions.items():
    ecu_key = (addr[0], addr[1])
    for version in versions:
      candidates = all_fw_versions[(*ecu_key, version)]
      if len(candidates) == 1:
        matched_ecus.add(ecu_key)
        if match is None:
          match = candidates[0]
        elif match != candidates[0]:
          return set()

  if match and len(matched_ecus) >= 2:
    if log:
      carlog.error(f"Fingerprinted {match} using fuzzy match. {len(matched_ecus)} matching ECUs")
    return {match}
  return set()


def match_fw_to_car_exact(live_fw_versions: LiveFwVersions, match_brand: str | None = None,
                          log: bool = True, extra_fw_versions: dict | None = None) -> set[str]:
  from opendbc.car.fingerprints import FW_VERSIONS
  from opendbc.car.fw_versions import FW_QUERY_CONFIGS, VERSIONS

  if extra_fw_versions is None:
    extra_fw_versions = {}

  matches = set()
  versions = VERSIONS if match_brand is None else {match_brand: VERSIONS[match_brand]}

  for brand, cars in versions.items():
    config = FW_QUERY_CONFIGS[brand]
    for candidate, fws in cars.items():
      for ecu, expected in fws.items():
        expected = expected + extra_fw_versions.get(candidate, {}).get(ecu, [])
        found = live_fw_versions.get(ecu[1:], set())
        if not found:
          if candidate in config.non_essential_ecus.get(ecu[0], []) or ecu[0] not in ESSENTIAL_ECUS:
            continue
        elif found & set(expected):
          continue
        break
      else:
        matches.add(candidate)

  return matches


def match_fw_to_car(fw_versions: list[CarParams.CarFw], vin: str, allow_exact: bool = True,
                    allow_fuzzy: bool = True, log: bool = True) -> tuple[bool, set[str]]:
  exact_matches: list[tuple[bool, MatchFwToCar]] = []
  if allow_exact:
    exact_matches = [(True, match_fw_to_car_exact)]
  if allow_fuzzy:
    exact_matches.append((False, match_fw_to_car_fuzzy))

  from opendbc.car.fw_versions import FW_QUERY_CONFIGS, VERSIONS

  for exact_match, match_func in exact_matches:
    matches: set[str] = set()
    for brand in VERSIONS.keys():
      fw_versions_dict = build_fw_dict(fw_versions, filter_brand=brand)
      matches |= match_func(fw_versions_dict, match_brand=brand, log=log)
      config = FW_QUERY_CONFIGS[brand]
      if not exact_match and not matches and config.match_fw_to_car_fuzzy is not None:
        matches |= config.match_fw_to_car_fuzzy(fw_versions_dict, vin, VERSIONS[brand])
    if len(matches):
      return exact_match, matches
  return True, set()


def get_present_ecus(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback, num_pandas: int = 1) -> set[EcuAddrBusType]:
  from opendbc.car.fw_versions import FW_QUERY_CONFIGS, VERSIONS
  queries, parallel_queries = {True: [], False: []}, {True: [], False: []}
  responses: set[EcuAddrBusType] = set()

  for brand, config in FW_QUERY_CONFIGS.items():
    for r in config.requests:
      if r.bus > num_pandas * 4 - 1:
        continue
      for ecu_type, addr, sub_addr in config.get_all_ecus(VERSIONS[brand]):
        if len(r.whitelist_ecus) == 0 or ecu_type in r.whitelist_ecus:
          a = (addr, sub_addr, r.bus)
          if sub_addr is None:
            if a not in parallel_queries[r.obd_multiplexing]:
              parallel_queries[r.obd_multiplexing].append(a)
          elif [a] not in queries[r.obd_multiplexing]:
            queries[r.obd_multiplexing].append([a])
          response_addr = uds.get_rx_addr_for_tx_addr(addr, r.rx_offset)
          responses.add((response_addr, sub_addr, r.bus))

  for obd_multiplexing in queries:
    queries[obd_multiplexing].insert(0, parallel_queries[obd_multiplexing])

  ecu_responses = set()
  for obd_multiplexing in queries:
    set_obd_multiplexing(obd_multiplexing)
    for query in queries[obd_multiplexing]:
      ecu_responses.update(get_ecu_addrs(can_recv, can_send, set(query), responses, timeout=0.1))
  return ecu_responses


def get_brand_ecu_matches(ecu_rx_addrs: set[EcuAddrBusType]) -> dict[str, list[bool]]:
  from opendbc.car.fw_versions import FW_QUERY_CONFIGS, VERSIONS
  brand_rx_addrs = {brand: set() for brand in FW_QUERY_CONFIGS}
  brand_matches = {brand: [] for brand in FW_QUERY_CONFIGS for _ in FW_QUERY_CONFIGS[brand].requests}

  for brand, config in FW_QUERY_CONFIGS.items():
    for r in config.requests:
      for ecu in config.get_all_ecus(VERSIONS[brand]):
        if len(r.whitelist_ecus) == 0 or ecu[0] in r.whitelist_ecus:
          brand_rx_addrs[brand].add((uds.get_rx_addr_for_tx_addr(ecu[1], r.rx_offset), ecu[2]))

  for brand, addrs in brand_rx_addrs.items():
    for addr in addrs:
      brand_matches[brand].append(addr in [addr[:2] for addr in ecu_rx_addrs])
  return brand_matches


def get_fw_versions_ordered(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback, vin: str,
                             ecu_rx_addrs: set[EcuAddrBusType], timeout: float = 0.1, num_pandas: int = 1, progress: bool = False) -> list[CarParams.CarFw]:
  all_car_fw = []
  brand_matches = get_brand_ecu_matches(ecu_rx_addrs)
  for brand in sorted(brand_matches, key=lambda b: (brand_matches[b].count(True), brand_matches[b].count(True) / len(brand_matches[b])), reverse=True):
    if True not in brand_matches[brand]:
      continue
    car_fw = get_fw_versions(can_recv, can_send, set_obd_multiplexing, query_brand=brand, timeout=timeout, num_pandas=num_pandas, progress=progress)
    all_car_fw.extend(car_fw)
    _, matches = match_fw_to_car(car_fw, vin, log=False)
    if len(matches) == 1:
      break
  return all_car_fw


def get_fw_versions(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback, query_brand: str | None = None,
                    extra: OfflineFwVersions | None = None, timeout: float = 0.1, num_pandas: int = 1, progress: bool = False) -> list[CarParams.CarFw]:
  from opendbc.car.fw_versions import FW_QUERY_CONFIGS, VERSIONS
  versions = VERSIONS.copy()
  if query_brand is not None:
    versions = {query_brand: versions[query_brand]}
  if extra is not None:
    versions.update(extra)

  addrs, parallel_addrs, ecu_types = [], [], {}
  for brand, brand_versions in versions.items():
    config = FW_QUERY_CONFIGS[brand]
    for ecu_type, addr, sub_addr in config.get_all_ecus(brand_versions):
      a = (brand, addr, sub_addr)
      if a not in ecu_types:
        ecu_types[a] = ecu_type
      if sub_addr is None:
        if a not in parallel_addrs:
          parallel_addrs.append(a)
      elif [a] not in addrs:
        addrs.append([a])
  addrs.insert(0, parallel_addrs)

  car_fw = []
  requests = [(brand, config, r) for brand, config in FW_QUERY_CONFIGS.items() for r in config.requests if is_brand(brand, query_brand)]
  from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
  for addr_group in addrs:
    for addr_chunk in chunks(addr_group):
      for brand, config, r in requests:
        if r.bus > num_pandas * 4 - 1:
          continue
        if r.bus % 4 == 1:
          set_obd_multiplexing(r.obd_multiplexing)
        try:
          query_addrs = [(a, s) for (b, a, s) in addr_chunk if b in (brand, 'any') and
                         (len(r.whitelist_ecus) == 0 or ecu_types[(b, a, s)] in r.whitelist_ecus)]
          if query_addrs:
            query = IsoTpParallelQuery(can_send, can_recv, r.bus, query_addrs, r.request, r.response, r.rx_offset)
            for (tx_addr, sub_addr), version in query.get_data(timeout).items():
              f = CarParams.CarFw()
              f.ecu = ecu_types.get((brand, tx_addr, sub_addr), Ecu.unknown)
              f.fwVersion = version
              f.address = tx_addr
              f.responseAddress = uds.get_rx_addr_for_tx_addr(tx_addr, r.rx_offset)
              f.request = r.request
              f.brand = brand
              f.bus = r.bus
              f.logging = r.logging or (f.ecu, tx_addr, sub_addr) in config.extra_ecus
              f.obdMultiplexing = r.obd_multiplexing
              if sub_addr is not None:
                f.subAddress = sub_addr
              car_fw.append(f)
        except Exception:
          carlog.exception("FW query exception")
  return car_fw
