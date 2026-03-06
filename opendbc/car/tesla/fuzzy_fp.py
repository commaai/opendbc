"""Tesla fuzzy fingerprinting implementation."""
import re
from opendbc.car.fw_query_definitions import LiveFwVersions, OfflineFwVersions
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

# Tesla FW version pattern examples:
# TeM3_E014p10_0.0.0 (16),E014.17.00        -> Model 3
# TeMYG4_Main_0.0.0 (77),Y4003.05.4          -> Model Y Gen4
# TeMX_SP_XP002p2_0.0.0 (23),XPR003.6.0      -> Model X

FW_PATTERN = re.compile(
  r'^Te'                           # Tesla prefix
  r'(?P<platform>[A-Z0-9]+?)_'     # Platform: M3, MYG4, MX
  r'(?P<variant>[A-Za-z0-9_]+)_'   # Variant: E014p10, Main, SP
  r'[\d.]+\s*\(\d+\),'             # Version with build
  r'(?P<ver>[A-Z][A-Z0-9.]+)$'     # Final version: E014.17.00, Y4003.05.4
)

def _parse_fw(fw: bytes) -> tuple[str | None, str | None]:
  """Extract platform and version prefix from FW string."""
  try:
    s = fw.decode('utf-8', errors='ignore')
    m = FW_PATTERN.match(s)
    if m:
      return m.group('platform'), m.group('ver')[0] if m.group('ver') else None
  except:
    pass
  return None, None

def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str,
                          offline_fw_versions: OfflineFwVersions) -> set[str]:
  """Fuzzy match Tesla firmware versions to car candidates."""
  candidates: set[str] = set()
  for candidate, fws in offline_fw_versions.items():
    expected_platforms: set[str] = set()
    expected_prefixes: set[str] = set()
    for ecu, versions in fws.items():
      for v in versions:
        plat, pref = _parse_fw(v)
        if plat:
          expected_platforms.add(plat)
        if pref:
          expected_prefixes.add(pref)
    for addr, live_versions in live_fw_versions.items():
      for lv in live_versions:
        live_plat, live_pref = _parse_fw(lv)
        if live_plat and live_plat in expected_platforms:
          candidates.add(candidate)
          break
        if live_pref and live_pref in expected_prefixes:
          candidates.add(candidate)
          break
      if candidate in candidates:
        break
  return candidates
