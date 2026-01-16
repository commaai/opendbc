import os
import time
from collections.abc import Mapping

FRAME_FINGERPRINT = 100  # 1s


# Lazy loading wrapper to defer interface imports until first access
class _LazyInterfaces(Mapping):
  _cache: dict | None = None

  def _load(self):
    if not self._cache:
      from opendbc.car.values import BRANDS
      self._cache = {m.value: __import__(f'opendbc.car.{b.__module__.split(".")[-2]}.interface', fromlist=['CarInterface']).CarInterface
                     for b in BRANDS for m in b}
    return self._cache

  def __getitem__(self, key): return self._load()[key]
  def __iter__(self): return iter(self._load())
  def __len__(self): return len(self._load())

# Lazy interface loading
interfaces = _LazyInterfaces()

def __getattr__(name):
  if name == 'interface_names':
    from opendbc.car.values import BRANDS
    return {b.__module__.split('.')[-2]: [m.value for m in b] for b in BRANDS}
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def can_fingerprint(can_recv):
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.fingerprints import all_legacy_fingerprint_cars, eliminate_incompatible_cars
  finger = gen_empty_fingerprint()
  candidate_cars = {i: all_legacy_fingerprint_cars() for i in [0, 1]}  # attempt fingerprint on both bus 0 and 1
  frame = 0
  startTime = time.monotonic()
  while True:
    for packet in can_recv():
      for msg in packet:
        if msg.src < 128:
          finger.setdefault(msg.src, {})[msg.address] = len(msg.dat)
        for bus in candidate_cars:
          if msg.src == bus and msg.address < 0x800 and msg.address not in (0x7df, 0x7e0, 0x7e8):
            candidate_cars[bus] = eliminate_incompatible_cars(msg, candidate_cars[bus])
      frame += 1

      matched_bus = next((bus for bus, cars in candidate_cars.items() if len(cars) == 1), None)
      if matched_bus is not None and frame >= FRAME_FINGERPRINT + 2:
        return candidate_cars[matched_bus][0], finger

      if all(len(cars) == 0 for cars in candidate_cars.values()) and frame >= FRAME_FINGERPRINT + 2:
        return None, finger

    if frame > 201:
      break

    if time.monotonic() - startTime > 1.0:
      break

  return None, finger


# **** for use live only ****
def fingerprint(can_recv, can_send, set_obd_multiplexing, num_pandas: int, cached_params=None):
  from opendbc.car.carlog import carlog
  from opendbc.car.structs import CarParams
  from opendbc.car.fw_versions import get_fw_versions_ordered, get_present_ecus, match_fw_to_car
  from opendbc.car.vin import get_vin, is_valid_vin, VIN_UNKNOWN

  fixed_fingerprint = os.environ.get('FINGERPRINT', "")
  skip_fw_query = os.environ.get('SKIP_FW_QUERY', False)
  disable_fw_cache = os.environ.get('DISABLE_FW_CACHE', False)

  if len(fixed_fingerprint):
    car_fingerprint = fixed_fingerprint
    source = CarParams.FingerprintSource.fixed
    car_fw = []
    vin = VIN_UNKNOWN
    finger = {i: {} for i in range(3)}
    exact_match = True
  elif cached_params is not None and not disable_fw_cache:
    car_fingerprint = cached_params.carFingerprint
    source = CarParams.FingerprintSource.fw
    car_fw = list(cached_params.carFw)
    vin = cached_params.carVin
    finger = {i: dict(f) for i, f in enumerate(cached_params.carFingerprintDict)}
    exact_match = True
  else:
    # 1. CAN fingerprinting returns a list of candidate cars
    car_fingerprint, finger = can_fingerprint(can_recv)
    source = CarParams.FingerprintSource.can
    vin = VIN_UNKNOWN
    car_fw = []
    exact_match = car_fingerprint is not None

    if not skip_fw_query:
      # 2. If no VIN is provided, query it
      vin = get_vin(can_recv, can_send, (0, 1))

      # 3. If multiple or no candidates, or VIN is unknown, query FW versions
      # If Skip fw query is set, we use CAN fingerprint or fixed
      if car_fingerprint is None or not is_valid_vin(vin) or not exact_match:
        # 3.1 Get present ECUs
        ecu_rx_addrs = get_present_ecus(can_recv, can_send, set_obd_multiplexing, num_pandas=num_pandas)
        # 3.2 Query FW versions from present ECUs
        car_fw = get_fw_versions_ordered(can_recv, can_send, set_obd_multiplexing, vin, ecu_rx_addrs, num_pandas=num_pandas)

        # 4. Corrected fingerprint from FW versions
        if len(car_fw):
          exact_match, matches = match_fw_to_car(car_fw, vin)
          if len(matches) == 1:
            car_fingerprint = list(matches)[0]
            source = CarParams.FingerprintSource.fw

  return car_fingerprint, finger, vin, car_fw, source, exact_match


  candidate, fingerprints, vin, car_fw, source, exact_match = fingerprint(can_recv, can_send, set_obd_multiplexing, num_pandas, cached_params)

  if candidate is None:
    carlog.error("car doesn't match")
    return None

  CarInterface = interfaces[candidate]
  CP = CarInterface.get_params(candidate, fingerprints, car_fw, alpha_long_allowed, is_release, vin=vin, any_fw=not exact_match)
  CP.carVin = vin
  CP.carFw = car_fw
  CP.fingerprintSource = source

  return CarInterface(CP)


def get_demo_car_params():
  from opendbc.car.mock.values import CAR as MOCK
  from opendbc.car.structs import CarParams

  platform = MOCK.MOCK
  CarInterface = interfaces[platform]
  CP = CarInterface.get_non_essential_params(platform)
  return CP
