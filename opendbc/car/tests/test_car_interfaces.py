import os
import math
import random
import unittest

from opendbc.car import DT_CTRL, CanData, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.fingerprints import FW_VERSIONS
from opendbc.car.fw_versions import FW_QUERY_CONFIGS
from opendbc.car.interfaces import CarInterfaceBase, get_interface_attr
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.values import PLATFORMS

ALL_ECUS = {ecu for ecus in FW_VERSIONS.values() for ecu in ecus.keys()}
ALL_ECUS |= {ecu for config in FW_QUERY_CONFIGS.values() for ecu in config.extra_ecus}

ALL_REQUESTS = {tuple(r.request) for config in FW_QUERY_CONFIGS.values() for r in config.requests}

# From panda/python/__init__.py
DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

ALL_ECUS_LIST = sorted(ALL_ECUS)
ALL_REQUESTS_LIST = sorted(ALL_REQUESTS)

MAX_EXAMPLES = int(os.environ.get('MAX_EXAMPLES', '15'))


def get_random_car_interface(car_name: str) -> CarInterfaceBase:
  # Fuzzy CAN fingerprints to test more states of the CarInterface
  fingerprint = {addr: random.choice(DLC_TO_LEN) for addr in random.sample(range(0x801), random.randint(0, 20))}
  fingerprints = {bus: fingerprint for bus in range(7)}

  # Random FW versions from possible ECUs
  car_fw = []
  for _ in range(random.randint(0, 10)):
    ecu = random.choice(ALL_ECUS_LIST)
    req = random.choice(ALL_REQUESTS_LIST)
    car_fw.append(structs.CarParams.CarFw(ecu=ecu[0], address=ecu[1], subAddress=ecu[2] or 0, request=req))

  CarInterface = interfaces[car_name]
  car_params = CarInterface.get_params(car_name, fingerprints, car_fw,
                                       alpha_long=random.choice([True, False]), is_release=False, docs=False)
  return CarInterface(car_params)


def _make_car_test(car_name):

  def test(self):
    for _ in range(MAX_EXAMPLES):
      car_interface = get_random_car_interface(car_name)
      car_params = car_interface.CP.as_reader()

      assert car_params.mass > 1
      assert car_params.wheelbase > 0
      # centerToFront is center of gravity to front wheels, assert a reasonable range
      assert car_params.wheelbase * 0.3 < car_params.centerToFront < car_params.wheelbase * 0.7
      assert car_params.maxLateralAccel > 0

      # Longitudinal sanity checks
      assert len(car_params.longitudinalTuning.kpV) == len(car_params.longitudinalTuning.kpBP)
      assert len(car_params.longitudinalTuning.kiV) == len(car_params.longitudinalTuning.kiBP)

      # Lateral sanity checks
      if car_params.steerControlType != structs.CarParams.SteerControlType.angle:
        tune = car_params.lateralTuning
        if tune.which() == 'pid':
          if car_name != MOCK.MOCK:
            assert not math.isnan(tune.pid.kf) and tune.pid.kf > 0
            assert len(tune.pid.kpV) > 0 and len(tune.pid.kpV) == len(tune.pid.kpBP)
            assert len(tune.pid.kiV) > 0 and len(tune.pid.kiV) == len(tune.pid.kiBP)

        elif tune.which() == 'torque':
          assert not math.isnan(tune.torque.latAccelFactor) and tune.torque.latAccelFactor > 0
          assert not math.isnan(tune.torque.friction) and tune.torque.friction > 0

      # Run car interface
      now_nanos = 0
      CC = structs.CarControl().as_reader()
      for _ in range(10):
        car_interface.update([])
        car_interface.apply(CC, now_nanos)
        now_nanos += DT_CTRL * 1e9  # 10 ms

      CC = structs.CarControl()
      CC.enabled = True
      CC.latActive = True
      CC.longActive = True
      CC = CC.as_reader()
      for _ in range(10):
        car_interface.update([])
        car_interface.apply(CC, now_nanos)
        now_nanos += DT_CTRL * 1e9  # 10ms

      # Test radar interface
      radar_interface = car_interface.RadarInterface(car_params)
      assert radar_interface

      # Run radar interface once
      radar_interface.update([])
      if not car_params.radarUnavailable and radar_interface.rcp is not None and \
         hasattr(radar_interface, '_update') and hasattr(radar_interface, 'trigger_msg'):
        radar_interface._update([radar_interface.trigger_msg])

      # Test radar fault
      if not car_params.radarUnavailable and radar_interface.rcp is not None:
        cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
        rr = radar_interface.update(cans)
        assert rr is None or len(rr.errors) > 0

  return test


class TestCarInterfaces(unittest.TestCase):
  def test_interface_attrs(self):
    """Asserts basic behavior of interface attribute getter"""
    num_brands = len(get_interface_attr('CAR'))
    assert num_brands >= 12

    # Should return value for all brands when not combining, even if attribute doesn't exist
    ret = get_interface_attr('FAKE_ATTR')
    assert len(ret) == num_brands

    # Make sure we can combine dicts
    ret = get_interface_attr('DBC', combine_brands=True)
    assert len(ret) >= 160

    # We don't support combining non-dicts
    ret = get_interface_attr('CAR', combine_brands=True)
    assert len(ret) == 0

    # If brand has None value, it shouldn't return when ignore_none=True is specified
    none_brands = {b for b, v in get_interface_attr('FINGERPRINTS').items() if v is None}
    assert len(none_brands) >= 1

    ret = get_interface_attr('FINGERPRINTS', ignore_none=True)
    none_brands_in_ret = none_brands.intersection(ret)
    assert len(none_brands_in_ret) == 0, f'Brands with None values in ignore_none=True result: {none_brands_in_ret}'


for car_name in sorted(PLATFORMS):
  setattr(TestCarInterfaces, f'test_car_interfaces_{car_name}', _make_car_test(car_name))
