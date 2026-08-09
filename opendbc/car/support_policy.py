from dataclasses import dataclass

from opendbc.car.docs_definitions import CarDocs
from opendbc.car.values import PLATFORMS
from opendbc.car.vin import VIN_UNKNOWN, is_valid_vin


VIN_MODEL_YEAR_CODES = {
  **dict(zip("ABCDEFGHJKLMNPRSTVWXY", range(2010, 2031), strict=True)),
  **dict(zip("123456789", range(2031, 2040), strict=True)),
}


@dataclass(frozen=True, order=True)
class SupportedVehicle:
  make: str
  model: str
  year: int


def get_vin_model_year(vin: str) -> int | None:
  """Decode the current (2010-2039) North American VIN year cycle."""
  if not is_valid_vin(vin) or vin == VIN_UNKNOWN:
    return None
  return VIN_MODEL_YEAR_CODES.get(vin[9])


def get_platform_whitelist(platform: str) -> frozenset[SupportedVehicle]:
  """Build the make/model/year support policy from documentation metadata."""
  config = PLATFORMS[platform].config
  docs = config.car_docs
  return frozenset(
    SupportedVehicle(doc.make, doc.model, int(year))
    for doc in docs if isinstance(doc, CarDocs)
    for year in doc.year_list
  )


def is_vehicle_supported(vehicle: SupportedVehicle, whitelist: frozenset[SupportedVehicle]) -> bool:
  """Strict make/model/year policy check for a fully decoded identity."""
  return vehicle in whitelist


def is_platform_vin_supported(platform: str, vin: str) -> bool:
  """Apply identity policy after protocol fingerprinting.

  Unknown VINs cannot be checked and preserve existing behavior. Make/model is
  temporarily supplied by the legacy identity; a Toyota VDS decoder can later
  narrow the candidate tuple without changing this policy representation.
  """
  year = get_vin_model_year(vin)
  if year is None:
    return True
  return any(vehicle.year == year for vehicle in get_platform_whitelist(platform))
