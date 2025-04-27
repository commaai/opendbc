import os
from enum import StrEnum
from typing import Any

from opendbc.car.common.basedir import BASEDIR

INTERFACE_EXT_ATTR_FILE = {
  "FW_VERSIONS_EXT": "fingerprints_ext",
}


def get_interface_ext_attr(attr: str, combine_brands: bool = False, ignore_none: bool = False) -> dict[str | StrEnum, Any]:
  # read all the folders in opendbc/car and return a dict where:
  # - keys are all the car models or brand names
  # - values are attr values from all car folders
  result = {}
  for car_folder in sorted([x[0] for x in os.walk(BASEDIR)]):
    try:
      brand_name = car_folder.split('/')[-1]
      brand_values = __import__(f'opendbc.sunnypilot.car.{brand_name}.{INTERFACE_EXT_ATTR_FILE.get(attr, "values")}', fromlist=[attr])
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


def merge_fw_versions(fw_versions):
  """
    Merge firmware versions by extending lists for matching ECUs,
    adding all entries regardless of duplicates.
  """
  FW_VERSIONS_EXT = get_interface_ext_attr('FW_VERSIONS_EXT', combine_brands=True, ignore_none=True)
  for c, f in FW_VERSIONS_EXT.items():
    if c not in fw_versions:
      fw_versions[c] = f
      continue

    for e, new_fw_list in f.items():
      if e not in fw_versions[c]:
        fw_versions[c][e] = new_fw_list
      else:
        fw_versions[c][e].extend(new_fw_list)

  return fw_versions
