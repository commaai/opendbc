from collections import defaultdict
from collections.abc import Callable, Iterator
from typing import Protocol, TypeVar

from tqdm import tqdm

from opendbc.car import uds
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.structs import CarParams
from opendbc.car.ecu_addrs import get_ecu_addrs
from opendbc.car.fingerprints import FW_VERSIONS
from opendbc.car.fw_query_definitions import ESSENTIAL_ECUS, AddrType, EcuAddrBusType, FwQueryConfig, LiveFwVersions, OfflineFwVersions
from opendbc.car.interfaces import get_interface_attr
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery

Ecu = CarParams.Ecu
FUZZY_EXCLUDE_ECUS = [Ecu.fwdCamera, Ecu.fwdRadar, Ecu.eps, Ecu.debug]

FW_QUERY_CONFIGS: dict[str, FwQueryConfig] = get_interface_attr('FW_QUERY_CONFIG', ignore_none=True)
VERSIONS = get_interface_attr('FW_VERSIONS', ignore_none=True)

MODEL_TO_BRAND = {c: b for b, e in VERSIONS.items() for c in e}
REQUESTS = [(brand, config, r) for brand, config in FW_QUERY_CONFIGS.items() for r in config.requests]

T = TypeVar('T')
ObdCallback = Callable[[bool], None]


def chunks(l: list[T], n: int = 128) -> Iterator[list[T]]:
  for i in range(0, len(l), n):
    yield l[i:i + n]


def is_brand(brand: str, filter_brand: str | None) -> bool:
  """Returns if brand matches filter_brand or no brand filter is specified"""
  return filter_brand is None or brand == filter_brand


def build_fw_dict(fw_versions: list[CarParams.CarFw], filter_brand: str = None) -> dict[AddrType, set[bytes]]:
  carlog.error(f"[DEBUG] build_fw_dict: filter_brand={filter_brand}, fw_versions count={len(fw_versions)}")
  fw_versions_dict: defaultdict[AddrType, set[bytes]] = defaultdict(set)
  for fw in fw_versions:
    carlog.error(f"[DEBUG] build_fw_dict: Processing FW - brand={fw.brand}, addr=0x{fw.address:X}, sub_addr={fw.subAddress}, logging={fw.logging}")
    if is_brand(fw.brand, filter_brand) and not fw.logging:
      sub_addr = fw.subAddress if fw.subAddress != 0 else None
      fw_versions_dict[(fw.address, sub_addr)].add(fw.fwVersion)
      carlog.error(f"[DEBUG] build_fw_dict: Added FW - addr=0x{fw.address:X}, sub_addr={sub_addr}, version={fw.fwVersion}")
    else:
      carlog.error(f"[DEBUG] build_fw_dict: Skipped FW - brand_match={is_brand(fw.brand, filter_brand)}, logging={fw.logging}")

  carlog.error(f"[DEBUG] build_fw_dict: Final dict has {len(fw_versions_dict)} entries")
  for addr, versions in fw_versions_dict.items():
    carlog.error(f"[DEBUG] build_fw_dict: Final entry - addr=0x{addr[0]:X}, sub_addr={addr[1]}, versions_count={len(versions)}")
  return dict(fw_versions_dict)


class MatchFwToCar(Protocol):
  def __call__(self, live_fw_versions: LiveFwVersions, match_brand: str = None, log: bool = True) -> set[str]:
    ...


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, match_brand: str = None, log: bool = True, exclude: str = None) -> set[str]:
  """Do a fuzzy FW match. This function will return a match, and the number of firmware version
  that were matched uniquely to that specific car. If multiple ECUs uniquely match to different cars
  the match is rejected."""

  carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: match_brand={match_brand}, exclude={exclude}")
  carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: live_fw_versions={live_fw_versions}")

  # Build lookup table from (addr, sub_addr, fw) to list of candidate cars
  all_fw_versions = defaultdict(list)
  for candidate, fw_by_addr in FW_VERSIONS.items():
    if not is_brand(MODEL_TO_BRAND[candidate], match_brand):
      continue

    if candidate == exclude:
      continue

    carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Processing candidate={candidate}")
    for addr, fws in fw_by_addr.items():
      # These ECUs are known to be shared between models (EPS only between hybrid/ICE version)
      # Getting this exactly right isn't crucial, but excluding camera and radar makes it almost
      # impossible to get 3 matching versions, even if two models with shared parts are released at the same
      # time and only one is in our database.
      if addr[0] in FUZZY_EXCLUDE_ECUS:
        carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Excluding ECU {addr[0]} for fuzzy matching")
        continue
      for f in fws:
        all_fw_versions[(addr[1], addr[2], f)].append(candidate)
        carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Added lookup - addr=0x{addr[1]:X}, sub_addr={addr[2]}, fw={f}, candidate={candidate}")

  matched_ecus = set()
  match: str | None = None
  for addr, versions in live_fw_versions.items():
    ecu_key = (addr[0], addr[1])
    carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Checking addr=0x{addr[0]:X}, sub_addr={addr[1]}, versions_count={len(versions)}")
    for version in versions:
      # All cars that have this FW response on the specified address
      candidates = all_fw_versions[(*ecu_key, version)]
      carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Version={version}, candidates={candidates}")

      if len(candidates) == 1:
        matched_ecus.add(ecu_key)
        carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Unique match found - ecu_key={ecu_key}, candidate={candidates[0]}")
        if match is None:
          match = candidates[0]
          carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Set match to {match}")
        # We uniquely matched two different cars. No fuzzy match possible
        elif match != candidates[0]:
          carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Conflicting matches - current={match}, new={candidates[0]}")
          return set()

  # Note that it is possible to match to a candidate without all its ECUs being present
  # if there are enough matches. FIXME: parameterize this or require all ECUs to exist like exact matching
  carlog.error(f"[DEBUG] match_fw_to_car_fuzzy: Final - match={match}, matched_ecus_count={len(matched_ecus)}")
  if match and len(matched_ecus) >= 2:
    if log:
      carlog.error(f"Fingerprinted {match} using fuzzy match. {len(matched_ecus)} matching ECUs")
    return {match}
  else:
    return set()


def match_fw_to_car_exact(live_fw_versions: LiveFwVersions, match_brand: str = None, log: bool = True, extra_fw_versions: dict = None) -> set[str]:
  """Do an exact FW match. Returns all cars that match the given
  FW versions for a list of "essential" ECUs. If an ECU is not considered
  essential the FW version can be missing to get a fingerprint, but if it's present it
  needs to match the database."""

  carlog.error(f"[DEBUG] match_fw_to_car_exact: match_brand={match_brand}")
  carlog.error(f"[DEBUG] match_fw_to_car_exact: live_fw_versions={live_fw_versions}")

  if extra_fw_versions is None:
    extra_fw_versions = {}

  invalid = set()
  candidates = {c: f for c, f in FW_VERSIONS.items() if
                is_brand(MODEL_TO_BRAND[c], match_brand)}

  carlog.error(f"[DEBUG] match_fw_to_car_exact: candidates={list(candidates.keys())}")

  for candidate, fws in candidates.items():
    carlog.error(f"[DEBUG] match_fw_to_car_exact: Checking candidate={candidate}")
    config = FW_QUERY_CONFIGS[MODEL_TO_BRAND[candidate]]
    for ecu, expected_versions in fws.items():
      expected_versions = expected_versions + extra_fw_versions.get(candidate, {}).get(ecu, [])
      ecu_type = ecu[0]
      addr = ecu[1:]

      carlog.error(f"[DEBUG] match_fw_to_car_exact: ECU - type={ecu_type}, addr=0x{addr[0]:X}, sub_addr={addr[1]}, expected_versions_count={len(expected_versions)}")

      found_versions = live_fw_versions.get(addr, set())
      carlog.error(f"[DEBUG] match_fw_to_car_exact: Found versions count={len(found_versions)}")

      if not len(found_versions):
        # Some models can sometimes miss an ecu, or show on two different addresses
        # FIXME: this logic can be improved to be more specific, should require one of the two addresses
        if candidate in config.non_essential_ecus.get(ecu_type, []):
          carlog.error(f"[DEBUG] match_fw_to_car_exact: Skipping non-essential ECU {ecu_type} for {candidate}")
          continue

        # Ignore non essential ecus
        if ecu_type not in ESSENTIAL_ECUS:
          carlog.error(f"[DEBUG] match_fw_to_car_exact: Skipping non-essential ECU type {ecu_type}")
          continue

      # Virtual debug ecu doesn't need to match the database
      if ecu_type == Ecu.debug:
        carlog.error(f"[DEBUG] match_fw_to_car_exact: Skipping debug ECU")
        continue

      version_match = any(found_version in expected_versions for found_version in found_versions)
      carlog.error(f"[DEBUG] match_fw_to_car_exact: Version match result={version_match}")

      if not version_match:
        carlog.error(f"[DEBUG] match_fw_to_car_exact: Invalid candidate {candidate} - no version match for ECU {ecu_type}")
        invalid.add(candidate)
        break

  result = set(candidates.keys()) - invalid
  carlog.error(f"[DEBUG] match_fw_to_car_exact: Final result={result}")
  return result


def match_fw_to_car(fw_versions: list[CarParams.CarFw], vin: str, allow_exact: bool = True,
                    allow_fuzzy: bool = True, log: bool = True) -> tuple[bool, set[str]]:

  carlog.error(f"[DEBUG] match_fw_to_car: vin={vin}, allow_exact={allow_exact}, allow_fuzzy={allow_fuzzy}")
  carlog.error(f"[DEBUG] match_fw_to_car: fw_versions count={len(fw_versions)}")

  # Try exact matching first
  exact_matches: list[tuple[bool, MatchFwToCar]] = []
  if allow_exact:
    exact_matches = [(True, match_fw_to_car_exact)]
  if allow_fuzzy:
    exact_matches.append((False, match_fw_to_car_fuzzy))

  for exact_match, match_func in exact_matches:
    carlog.error(f"[DEBUG] match_fw_to_car: Trying exact_match={exact_match}")
    # For each brand, attempt to fingerprint using all FW returned from its queries
    matches: set[str] = set()
    for brand in VERSIONS.keys():
      carlog.error(f"[DEBUG] match_fw_to_car: Trying brand={brand}")
      fw_versions_dict = build_fw_dict(fw_versions, filter_brand=brand)
      brand_matches = match_func(fw_versions_dict, match_brand=brand, log=log)
      carlog.error(f"[DEBUG] match_fw_to_car: Brand {brand} matches={brand_matches}")
      matches |= brand_matches

      # If specified and no matches so far, fall back to brand's fuzzy fingerprinting function
      config = FW_QUERY_CONFIGS[brand]
      if not exact_match and not len(matches) and config.match_fw_to_car_fuzzy is not None:
        carlog.error(f"[DEBUG] match_fw_to_car: Trying custom fuzzy function for brand {brand}")
        custom_matches = config.match_fw_to_car_fuzzy(fw_versions_dict, vin, VERSIONS[brand])
        carlog.error(f"[DEBUG] match_fw_to_car: Custom fuzzy matches={custom_matches}")
        matches |= custom_matches

    carlog.error(f"[DEBUG] match_fw_to_car: Total matches for exact_match={exact_match}: {matches}")
    if len(matches):
      return exact_match, matches

  carlog.error(f"[DEBUG] match_fw_to_car: No matches found")
  return True, set()


def get_present_ecus(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback, num_pandas: int = 1) -> set[EcuAddrBusType]:
  carlog.error(f"[DEBUG] get_present_ecus: Starting with num_pandas={num_pandas}")

  # queries are split by OBD multiplexing mode
  queries: dict[bool, list[list[EcuAddrBusType]]] = {True: [], False: []}
  parallel_queries: dict[bool, list[EcuAddrBusType]] = {True: [], False: []}
  responses: set[EcuAddrBusType] = set()

  carlog.error(f"[DEBUG] get_present_ecus: Total REQUESTS count={len(REQUESTS)}")
  for i, (brand, config, r) in enumerate(REQUESTS):
    carlog.error(f"[DEBUG] get_present_ecus: Request {i}: brand={brand}, bus={r.bus}, whitelist={r.whitelist_ecus}, obd_multiplexing={r.obd_multiplexing}")

    # Skip query if no panda available
    if r.bus > num_pandas * 4 - 1:
      carlog.error(f"[DEBUG] get_present_ecus: Skipping request {i} - bus {r.bus} > available buses")
      continue

    ecu_count = 0
    for ecu_type, addr, sub_addr in config.get_all_ecus(VERSIONS[brand]):
      ecu_count += 1
      carlog.error(f"[DEBUG] get_present_ecus: ECU {ecu_count} - type={ecu_type}, addr=0x{addr:X}, sub_addr={sub_addr}")

      # Only query ecus in whitelist if whitelist is not empty
      if len(r.whitelist_ecus) == 0 or ecu_type in r.whitelist_ecus:
        carlog.error(f"[DEBUG] get_present_ecus: ECU {ecu_count} passes whitelist check")
        a = (addr, sub_addr, r.bus)
        # Build set of queries
        if sub_addr is None:
          if a not in parallel_queries[r.obd_multiplexing]:
            parallel_queries[r.obd_multiplexing].append(a)
            carlog.error(f"[DEBUG] get_present_ecus: Added to parallel_queries[{r.obd_multiplexing}]: {a}")
        else:  # subaddresses must be queried one by one
          if [a] not in queries[r.obd_multiplexing]:
            queries[r.obd_multiplexing].append([a])
            carlog.error(f"[DEBUG] get_present_ecus: Added to queries[{r.obd_multiplexing}]: [{a}]")

        # Build set of expected responses to filter
        response_addr = uds.get_rx_addr_for_tx_addr(addr, r.rx_offset)
        response_tuple = (response_addr, sub_addr, r.bus)
        responses.add(response_tuple)
        carlog.error(f"[DEBUG] get_present_ecus: Added expected response: 0x{response_addr:X}, sub_addr={sub_addr}, bus={r.bus}")
      else:
        carlog.error(f"[DEBUG] get_present_ecus: ECU {ecu_count} failed whitelist check - type={ecu_type} not in {r.whitelist_ecus}")

  for obd_multiplexing in queries:
    queries[obd_multiplexing].insert(0, parallel_queries[obd_multiplexing])

  carlog.error(f"[DEBUG] get_present_ecus: Final queries structure:")
  for obd_mode, query_list in queries.items():
    carlog.error(f"[DEBUG] get_present_ecus: OBD {obd_mode}: {len(query_list)} query groups")
    for i, query_group in enumerate(query_list):
      carlog.error(f"[DEBUG] get_present_ecus: Group {i}: {query_group}")

  carlog.error(f"[DEBUG] get_present_ecus: Expected responses ({len(responses)}):")
  for resp in responses:
    carlog.error(f"[DEBUG] get_present_ecus: Expected: 0x{resp[0]:X}, sub_addr={resp[1]}, bus={resp[2]}")

  ecu_responses = set()
  for obd_multiplexing in queries:
    carlog.error(f"[DEBUG] get_present_ecus: Setting OBD multiplexing to {obd_multiplexing}")
    set_obd_multiplexing(obd_multiplexing)
    for i, query in enumerate(queries[obd_multiplexing]):
      carlog.error(f"[DEBUG] get_present_ecus: Executing query group {i}: {query}")
      query_responses = get_ecu_addrs(can_recv, can_send, set(query), responses, timeout=0.1)
      carlog.error(f"[DEBUG] get_present_ecus: Query group {i} responses: {query_responses}")
      ecu_responses.update(query_responses)

  carlog.error(f"[DEBUG] get_present_ecus: Final ECU responses ({len(ecu_responses)}):")
  for resp in ecu_responses:
    carlog.error(f"[DEBUG] get_present_ecus: Found: 0x{resp[0]:X}, sub_addr={resp[1]}, bus={resp[2]}")

  return ecu_responses


def get_brand_ecu_matches(ecu_rx_addrs: set[EcuAddrBusType]) -> dict[str, list[bool]]:
  """Returns dictionary of brands and matches with ECUs in their FW versions"""

  carlog.error(f"[DEBUG] get_brand_ecu_matches: Input ecu_rx_addrs ({len(ecu_rx_addrs)}):")
  for addr in ecu_rx_addrs:
    carlog.error(f"[DEBUG] get_brand_ecu_matches: Input addr: 0x{addr[0]:X}, sub_addr={addr[1]}, bus={addr[2]}")

  brand_rx_addrs = {brand: set() for brand in FW_QUERY_CONFIGS}
  brand_matches = {brand: [] for brand, _, _ in REQUESTS}

  carlog.error(f"[DEBUG] get_brand_ecu_matches: Processing {len(REQUESTS)} requests")

  # Since we can't know what request an ecu responded to, add matches for all possible rx offsets
  for i, (brand, config, r) in enumerate(REQUESTS):
    carlog.error(f"[DEBUG] get_brand_ecu_matches: Request {i}: brand={brand}, bus={r.bus}, rx_offset={r.rx_offset}, whitelist={r.whitelist_ecus}")

    ecu_count = 0
    for ecu in config.get_all_ecus(VERSIONS[brand]):
      ecu_count += 1
      ecu_type, addr, sub_addr = ecu
      carlog.error(f"[DEBUG] get_brand_ecu_matches: Request {i}, ECU {ecu_count}: type={ecu_type}, addr=0x{addr:X}, sub_addr={sub_addr}")

      if len(r.whitelist_ecus) == 0 or ecu_type in r.whitelist_ecus:
        rx_addr = uds.get_rx_addr_for_tx_addr(addr, r.rx_offset)
        expected_addr = (rx_addr, sub_addr)
        brand_rx_addrs[brand].add(expected_addr)
        carlog.error(f"[DEBUG] get_brand_ecu_matches: Request {i}, ECU {ecu_count}: Added expected addr 0x{rx_addr:X}, sub_addr={sub_addr} to brand {brand}")
      else:
        carlog.error(f"[DEBUG] get_brand_ecu_matches: Request {i}, ECU {ecu_count}: Skipped due to whitelist - {ecu_type} not in {r.whitelist_ecus}")

  carlog.error(f"[DEBUG] get_brand_ecu_matches: Brand expected addresses:")
  for brand, addrs in brand_rx_addrs.items():
    carlog.error(f"[DEBUG] get_brand_ecu_matches: Brand {brand} expects {len(addrs)} addresses:")
    for addr in addrs:
      carlog.error(f"[DEBUG] get_brand_ecu_matches: Brand {brand}: 0x{addr[0]:X}, sub_addr={addr[1]}")

  # Convert ecu_rx_addrs to just (addr, sub_addr) for comparison
  actual_addrs = [addr[:2] for addr in ecu_rx_addrs]
  carlog.error(f"[DEBUG] get_brand_ecu_matches: Actual addresses for comparison ({len(actual_addrs)}):")
  for addr in actual_addrs:
    carlog.error(f"[DEBUG] get_brand_ecu_matches: Actual: 0x{addr[0]:X}, sub_addr={addr[1]}")

  for brand, addrs in brand_rx_addrs.items():
    carlog.error(f"[DEBUG] get_brand_ecu_matches: Checking matches for brand {brand}")
    for addr in addrs:
      # TODO: check bus from request as well
      match_found = addr in actual_addrs
      brand_matches[brand].append(match_found)
      carlog.error(f"[DEBUG] get_brand_ecu_matches: Brand {brand}, addr 0x{addr[0]:X}: match={match_found}")

  carlog.error(f"[DEBUG] get_brand_ecu_matches: Final brand matches:")
  for brand, matches in brand_matches.items():
    true_count = matches.count(True)
    total_count = len(matches)
    carlog.error(f"[DEBUG] get_brand_ecu_matches: Brand {brand}: {true_count}/{total_count} matches, list={matches}")

  return brand_matches


def get_fw_versions_ordered(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback, vin: str,
                            ecu_rx_addrs: set[EcuAddrBusType], timeout: float = 0.1, num_pandas: int = 1, progress: bool = False) -> list[CarParams.CarFw]:
  """Queries for FW versions ordering brands by likelihood, breaks when exact match is found"""

  carlog.error(f"[DEBUG] get_fw_versions_ordered: Starting with vin={vin}, timeout={timeout}, num_pandas={num_pandas}")
  carlog.error(f"[DEBUG] get_fw_versions_ordered: ecu_rx_addrs ({len(ecu_rx_addrs)}):")
  for addr in ecu_rx_addrs:
    carlog.error(f"[DEBUG] get_fw_versions_ordered: Input addr: 0x{addr[0]:X}, sub_addr={addr[1]}, bus={addr[2]}")

  all_car_fw = []
  brand_matches = get_brand_ecu_matches(ecu_rx_addrs)

  carlog.error(f"[DEBUG] get_fw_versions_ordered: Brand matches summary:")
  for brand, matches in brand_matches.items():
    true_count = matches.count(True)
    total_count = len(matches)
    percentage = (true_count / total_count * 100) if total_count > 0 else 0
    carlog.error(f"[DEBUG] get_fw_versions_ordered: Brand {brand}: {true_count}/{total_count} ({percentage:.1f}%)")

  # Sort brands by number of matching ECUs first, then percentage of matching ECUs in the database
  # This allows brands with only one ECU to be queried first (e.g. Tesla)
  sorted_brands = sorted(brand_matches, key=lambda b: (brand_matches[b].count(True), brand_matches[b].count(True) / len(brand_matches[b])), reverse=True)
  carlog.error(f"[DEBUG] get_fw_versions_ordered: Sorted brands: {sorted_brands}")

  for brand in sorted_brands:
    carlog.error(f"[DEBUG] get_fw_versions_ordered: Processing brand {brand}")
    # Skip this brand if there are no matching present ECUs
    if True not in brand_matches[brand]:
      carlog.error(f"[DEBUG] get_fw_versions_ordered: SKIPPING brand {brand} - no matching ECUs")
      continue

    carlog.error(f"[DEBUG] get_fw_versions_ordered: Querying FW for brand {brand}")
    car_fw = get_fw_versions(can_recv, can_send, set_obd_multiplexing, query_brand=brand, timeout=timeout, num_pandas=num_pandas, progress=progress)
    carlog.error(f"[DEBUG] get_fw_versions_ordered: Brand {brand} returned {len(car_fw)} FW entries")
    all_car_fw.extend(car_fw)

    # If there is a match using this brand's FW alone, finish querying early
    _, matches = match_fw_to_car(car_fw, vin, log=False)
    carlog.error(f"[DEBUG] get_fw_versions_ordered: Brand {brand} match result: {matches}")
    if len(matches) == 1:
      carlog.error(f"[DEBUG] get_fw_versions_ordered: Found unique match {matches}, breaking early")
      break

  carlog.error(f"[DEBUG] get_fw_versions_ordered: Final result: {len(all_car_fw)} total FW entries")
  return all_car_fw

def get_fw_versions(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback, query_brand: str = None,
                    extra: OfflineFwVersions = None, timeout: float = 0.1, num_pandas: int = 1, progress: bool = False) -> list[CarParams.CarFw]:

  carlog.error(f"[DEBUG] get_fw_versions: query_brand={query_brand}, timeout={timeout}, num_pandas={num_pandas}")

  versions = VERSIONS.copy()

  if query_brand is not None:
    versions = {query_brand: versions[query_brand]}
    carlog.error(f"[DEBUG] get_fw_versions: Filtered to single brand: {query_brand}")

  if extra is not None:
    versions.update(extra)
    carlog.error(f"[DEBUG] get_fw_versions: Added extra versions")

  carlog.error(f"[DEBUG] get_fw_versions: Working with brands: {list(versions.keys())}")

  # Extract ECU addresses to query from fingerprints
  # ECUs using a subaddress need be queried one by one, the rest can be done in parallel
  addrs = []
  parallel_addrs = []
  ecu_types = {}

  for brand, brand_versions in versions.items():
    carlog.error(f"[DEBUG] get_fw_versions: Processing brand {brand}")
    config = FW_QUERY_CONFIGS[brand]
    ecu_count = 0
    for ecu_type, addr, sub_addr in config.get_all_ecus(brand_versions):
      ecu_count += 1
      a = (brand, addr, sub_addr)
      if a not in ecu_types:
        ecu_types[a] = ecu_type

      carlog.error(f"[DEBUG] get_fw_versions: Brand {brand}, ECU {ecu_count}: type={ecu_type}, addr=0x{addr:X}, sub_addr={sub_addr}")

      if sub_addr is None:
        if a not in parallel_addrs:
          parallel_addrs.append(a)
          carlog.error(f"[DEBUG] get_fw_versions: Added to parallel_addrs: {a}")
      else:
        if [a] not in addrs:
          addrs.append([a])
          carlog.error(f"[DEBUG] get_fw_versions: Added to addrs: [{a}]")

  addrs.insert(0, parallel_addrs)
  carlog.error(f"[DEBUG] get_fw_versions: Final addrs structure: {len(addrs)} groups")
  for i, addr_group in enumerate(addrs):
    carlog.error(f"[DEBUG] get_fw_versions: Group {i}: {len(addr_group)} addresses")

  # Get versions and build capnp list to put into CarParams
  car_fw = []
  requests = [(brand, config, r) for brand, config, r in REQUESTS if is_brand(brand, query_brand)]
  carlog.error(f"[DEBUG] get_fw_versions: Total requests to process: {len(requests)}")

  for i, addr_group in enumerate(tqdm(addrs, disable=not progress)):  # split by subaddr, if any
    carlog.error(f"[DEBUG] get_fw_versions: Processing address group {i} with {len(addr_group)} addresses")

    for j, addr_chunk in enumerate(chunks(addr_group)):
      carlog.error(f"[DEBUG] get_fw_versions: Processing chunk {j} with {len(addr_chunk)} addresses")

      for k, (brand, config, r) in enumerate(requests):
        carlog.error(f"[DEBUG] get_fw_versions: Processing request {k}: brand={brand}, bus={r.bus}, rx_offset={r.rx_offset}")

        # Skip query if no panda available
        if r.bus > num_pandas * 4 - 1:
          carlog.error(f"[DEBUG] get_fw_versions: Skipping request {k} - bus {r.bus} > available buses ({num_pandas * 4 - 1})")
          continue

        # Toggle OBD multiplexing for each request
        if r.bus % 4 == 1:
          carlog.error(f"[DEBUG] get_fw_versions: Setting OBD multiplexing to {r.obd_multiplexing} for bus {r.bus}")
          set_obd_multiplexing(r.obd_multiplexing)

        try:
          query_addrs = [(a, s) for (b, a, s) in addr_chunk if b in (brand, 'any') and
                         (len(r.whitelist_ecus) == 0 or ecu_types[(b, a, s)] in r.whitelist_ecus)]

          carlog.error(f"[DEBUG] get_fw_versions: Request {k} query_addrs: {len(query_addrs)} addresses")
          for addr_idx, (addr, sub_addr) in enumerate(query_addrs):
            carlog.error(f"[DEBUG] get_fw_versions: Query addr {addr_idx}: 0x{addr:X}, sub_addr={sub_addr}")

          if query_addrs:
            carlog.error(f"[DEBUG] get_fw_versions: Creating IsoTpParallelQuery for request {k}")
            query = IsoTpParallelQuery(can_send, can_recv, r.bus, query_addrs, r.request, r.response, r.rx_offset)

            carlog.error(f"[DEBUG] get_fw_versions: Executing query with timeout={timeout}")
            query_results = query.get_data(timeout)
            carlog.error(f"[DEBUG] get_fw_versions: Query returned {len(query_results)} results")

            for result_idx, ((tx_addr, sub_addr), version) in enumerate(query_results.items()):
              carlog.error(f"[DEBUG] get_fw_versions: Result {result_idx}: tx_addr=0x{tx_addr:X}, sub_addr={sub_addr}, version={version}")

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
              carlog.error(f"[DEBUG] get_fw_versions: Added CarFw entry - ecu={f.ecu}, addr=0x{f.address:X}, brand={f.brand}, logging={f.logging}")
          else:
            carlog.error(f"[DEBUG] get_fw_versions: No query addresses for request {k}")

        except Exception as e:
          carlog.error(f"[DEBUG] get_fw_versions: Exception in request {k}: {str(e)}")
          carlog.exception("FW query exception")

  carlog.error(f"[DEBUG] get_fw_versions: Completed - returning {len(car_fw)} CarFw entries")
  return car_fw
