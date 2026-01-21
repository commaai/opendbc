import pytest
from types import SimpleNamespace

@pytest.fixture(scope="module")
def car_lib():
  """
  Lazy-loads car library modules to avoid paying the import cost
  during test collection. Returns a namespace-like object.
  """
  import numpy as np
  from opendbc.car import structs as struct, gen_empty_fingerprint, car_helpers, vin
  from opendbc.car import can_definitions, interfaces as interfaces_module
  from opendbc.car import fw_versions, fingerprints, docs
  from opendbc.car import vehicle_model
  from opendbc import car_discovery as discovery

  from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car, get_brand_ecu_matches

  return SimpleNamespace(
    np=np,
    struct=struct,
    gen_empty_fingerprint=gen_empty_fingerprint,
    car_helpers=car_helpers,
    vin=vin,
    can_definitions=can_definitions,
    interfaces_module=interfaces_module,
    fw_versions=fw_versions,
    fingerprints=fingerprints,
    docs=docs,
    vehicle_model=vehicle_model,
    discovery=discovery,
    build_fw_dict=build_fw_dict,
    match_fw_to_car=match_fw_to_car,
    get_brand_ecu_matches=get_brand_ecu_matches,
    CanData=can_definitions.CanData,
    CarParams=struct.CarParams,
    interfaces=car_helpers.interfaces,
    FW_VERSIONS=fingerprints.FW_VERSIONS,
    get_all_car_names=car_helpers.interfaces.get_all_car_names,
  )
