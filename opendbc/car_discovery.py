import re
import os
from functools import lru_cache
from typing import Any
from enum import StrEnum

# We calculate BASEDIR relative to this file (opendbc/car_discovery.py) -> opendbc/car
BASEDIR = os.path.abspath(os.path.join(os.path.dirname(__file__), 'car'))

INTERFACE_ATTR_FILE = {
  "FINGERPRINTS": "fingerprints",
  "FW_VERSIONS": "fingerprints",
  "FW_QUERY_CONFIG": "values",
}

@lru_cache(maxsize=None)
def get_all_car_names():
  """
  Scans the opendbc/car directory and used regex to find all
  CAR class attributes (car model names) without importing the modules.
  This drastically reduces pytest collection time by avoiding eager imports.
  """
  car_names = set()

  if not os.path.exists(BASEDIR):
    return []

  # Regex to match: "  MODEL_NAME =" or "  MODEL_NAME: Type ="
  req_pattern = re.compile(r'^\s+([A-Z0-9_]+)(\s*:\s*[^=]+)?\s*=')

  # Scan all brand directories
  for entry in os.scandir(BASEDIR):
    if not entry.is_dir():
      continue

    values_path = os.path.join(entry.path, 'values.py')
    if not os.path.exists(values_path):
      continue

    try:
      with open(values_path, 'r') as f:
        in_car_class = False
        for line in f:
          if line.lstrip().startswith('class CAR'):
            in_car_class = True
            continue
          if in_car_class and line.startswith('class '):
            in_car_class = False
            break

          if in_car_class:
            m = req_pattern.match(line)
            if m:
              name = m.group(1)
              if not name.startswith('_'):
                car_names.add(name)
    except OSError:
      pass

  return sorted(list(car_names))


def get_interface_attr(attr: str, combine_brands: bool = False, ignore_none: bool = False) -> dict[str | StrEnum, Any]:
  # read all the folders in opendbc/car and return a dict where:
  # - keys are all the car models or brand names
  # - values are attr values from all car folders
  result = {}
  for entry in sorted(os.scandir(BASEDIR), key=lambda x: x.name):
    if not entry.is_dir():
      continue

    try:
      brand_name = entry.name
      brand_values = __import__(f'opendbc.car.{brand_name}.{INTERFACE_ATTR_FILE.get(attr, "values")}', fromlist=[attr])
      if hasattr(brand_values, attr) or not ignore_none:
        attr_data = getattr(brand_values, attr, None)
      else:
        continue

      if combine_brands:
        if isinstance(attr_data, dict):
          for f, v in attr_data.items():
            result[f] = v
      else:
        result[brand_name] = attr_data
    except (ImportError, OSError):
      pass

  return result
