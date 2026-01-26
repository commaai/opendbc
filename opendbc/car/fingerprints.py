from opendbc.car_discovery import get_interface_attr


def __dir__():
  return sorted(
    list(globals().keys())
    + ["FW_VERSIONS", "_FINGERPRINTS", "BODY", "CHRYSLER", "FORD", "GM", "HONDA", "HYUNDAI", "MAZDA", "MOCK", "NISSAN", "SUBARU", "TOYOTA", "VW"]
  )


_DEBUG_ADDRESS = {1880: 8}  # reserved for debug purposes


def is_valid_for_fingerprint(msg, car_fingerprint: dict[int, int]):
  adr = msg.address
  # ignore addresses that are more than 11 bits
  return (adr in car_fingerprint and car_fingerprint[adr] == len(msg.dat)) or adr >= 0x800


def eliminate_incompatible_cars(msg, candidate_cars):
  """Removes cars that could not have sent msg.

  Inputs:
   msg: A cereal/log CanData message from the car.
   candidate_cars: A list of cars to consider.

  Returns:
   A list containing the subset of candidate_cars that could have sent msg.
  """
  from opendbc.car.fingerprints import _FINGERPRINTS

  compatible_cars = []

  for car_name in candidate_cars:
    car_fingerprints = _FINGERPRINTS[car_name]

    for fingerprint in car_fingerprints:
      # add alien debug address
      if is_valid_for_fingerprint(msg, fingerprint | _DEBUG_ADDRESS):
        compatible_cars.append(car_name)
        break

  return compatible_cars


def all_legacy_fingerprint_cars():
  """Returns a list of all known car strings, FPv1 only."""
  from opendbc.car.fingerprints import _FINGERPRINTS

  return list(_FINGERPRINTS.keys())


# A dict that maps old platform strings to their latest representations
def __getattr_migration__(name):
  if name == "MIGRATION":
    # Use a simple string-to-string mapping
    val = {
      "mock": "mock",
      # Common old platform names that might be referenced by commaCarSegments
      "COMMA BODY": "body.COMMA_BODY",
      "CHRYSLER PACIFICA HYBRID 2017": "chrysler.CHRYSLER_PACIFICA_2018_HYBRID",
      "CHRYSLER PACIFICA HYBRID 2018": "chrysler.CHRYSLER_PACIFICA_2018_HYBRID",
      "CHRYSLER PACIFICA HYBRID 2019": "chrysler.CHRYSLER_PACIFICA_2019_HYBRID",
      "CHRYSLER PACIFICA 2018": "chrysler.CHRYSLER_PACIFICA_2018",
      "CHRYSLER PACIFICA 2020": "chrysler.CHRYSLER_PACIFICA_2020",
      "HONDA ACCORD 2018": "body.HONDA_ACCORD",
      "HONDA CIVIC 2016": "body.HONDA_CIVIC",
      "HONDA CR-V 2016": "body.HONDA_CRV",
      "TOYOTA CAMRY 2018": "toyota.TOYOTA_CAMRY",
      "TOYOTA PRIUS 2017": "toyota.TOYOTA_PRIUS",
      "TOYOTA RAV4 2017": "toyota.TOYOTA_RAV4",
      "NISSAN LEAF 2018": "nissan.NISSAN_LEAF",
    }
    globals()[name] = val
    return val
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __getattr__(name):
  if name == "MIGRATION":
    return __getattr_migration__(name)
  elif name == "FW_VERSIONS":
    val = get_interface_attr("FW_VERSIONS", combine_brands=True, ignore_none=True)
  elif name == "_FINGERPRINTS":
    val = get_interface_attr("FINGERPRINTS", combine_brands=True, ignore_none=True)
  elif name in (
    n := {
      "BODY": "body",
      "CHRYSLER": "chrysler",
      "FORD": "ford",
      "GM": "gm",
      "HONDA": "honda",
      "HYUNDAI": "hyundai",
      "MAZDA": "mazda",
      "MOCK": "mock",
      "NISSAN": "nissan",
      "SUBARU": "subaru",
      "TOYOTA": "toyota",
      "VW": "volkswagen",
    }
  ):
    val = __import__(f"opendbc.car.{n[name]}.values", fromlist=["CAR"]).CAR
  else:
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

  globals()[name] = val
  return val
