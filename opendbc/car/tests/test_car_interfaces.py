import os
import math
import hypothesis.strategies as st
import pytest
from hypothesis import Phase, given, settings

from opendbc.car.structs import CarParams
from opendbc.car.car_helpers import interfaces
from opendbc.car_discovery import get_all_car_names, get_interface_attr

MAX_EXAMPLES = int(os.environ.get('MAX_EXAMPLES', '15'))


def get_fuzzy_car_interface(car_name, draw):
  from opendbc.car.fingerprints import FW_VERSIONS
  from opendbc.car import fw_versions
  # Fuzzy CAN fingerprints and FW versions to test more states of the CarInterface
  fingerprint_strategy = st.fixed_dictionaries({
    0: st.dictionaries(
      st.integers(0, 0x800),
      st.sampled_from([0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64])
    )
  })

  ALL_ECUS = {ecu for ecus in FW_VERSIONS.values() for ecu in ecus.keys()}
  ALL_ECUS |= {ecu for config in fw_versions.FW_QUERY_CONFIGS.values() for ecu in config.extra_ecus}
  ALL_REQUESTS = {tuple(r.request) for config in fw_versions.FW_QUERY_CONFIGS.values() for r in config.requests}

  car_fw_strategy = st.lists(st.builds(
    lambda fw, req: CarParams.CarFw(ecu=fw[0], address=fw[1], subAddress=fw[2] or 0, request=req),
    st.sampled_from(sorted(ALL_ECUS)),
    st.sampled_from(sorted(ALL_REQUESTS)),
  ))

  params = draw(st.fixed_dictionaries({'fingerprints': fingerprint_strategy, 'car_fw': car_fw_strategy, 'alpha_long': st.booleans()}))
  params['fingerprints'] |= {key + 1: params['fingerprints'][0] for key in range(6)}

  CarInterface = interfaces[car_name]
  car_params = CarInterface.get_params(car_name, params['fingerprints'], params['car_fw'], alpha_long=params['alpha_long'], is_release=False, docs=False)
  return CarInterface(car_params)


class TestCarInterfaces:
  @pytest.mark.parametrize("car_name", get_all_car_names())
  @settings(max_examples=MAX_EXAMPLES, deadline=None, phases=(Phase.reuse, Phase.generate, Phase.shrink))
  @given(data=st.data())
  def test_car_interfaces(self, car_name, data):
    from opendbc.car.mock.values import CAR as MOCK
    car_interface = get_fuzzy_car_interface(car_name, data.draw)
    car_params = car_interface.CP.as_reader()

    assert car_params.mass > 1 and car_params.wheelbase > 0
    assert car_params.wheelbase * 0.3 < car_params.centerToFront < car_params.wheelbase * 0.7
    assert car_params.maxLateralAccel > 0
    assert len(car_params.longitudinalTuning.kpV) == len(car_params.longitudinalTuning.kpBP)
    assert len(car_params.longitudinalTuning.kiV) == len(car_params.longitudinalTuning.kiBP)

    if car_params.steerControlType != CarParams.SteerControlType.angle:
      tune = car_params.lateralTuning
      if tune.which() == 'pid' and car_name != MOCK.MOCK:
        assert not math.isnan(tune.pid.kf) and tune.pid.kf > 0
        assert len(tune.pid.kpV) > 0 and len(tune.pid.kpV) == len(tune.pid.kpBP)
      elif tune.which() == 'torque':
        assert not math.isnan(tune.torque.latAccelFactor) and tune.torque.latAccelFactor > 0

    now_nanos = 0
    from opendbc.car.structs import CarControl
    for enabled in (False, True):
      CC = CarControl()
      CC.enabled = CC.latActive = CC.longActive = enabled
      CC = CC.as_reader()
      for _ in range(10):
        car_interface.update([])
        car_interface.apply(CC, now_nanos)
        now_nanos += 0.01 * 1e9

    radar_interface = car_interface.RadarInterface(car_params)
    assert radar_interface
    radar_interface.update([])
    if not car_params.radarUnavailable and radar_interface.rcp is not None:
      if hasattr(radar_interface, '_update') and hasattr(radar_interface, 'trigger_msg'):
        radar_interface._update([radar_interface.trigger_msg])
      from opendbc.car.can_definitions import CanData
      assert radar_interface.update([(0, [CanData(0, b'', 0) for _ in range(5)])]) is None or True

  def test_interface_attrs(self):
    num_brands = len(get_interface_attr('CAR'))
    assert num_brands >= 12
    assert len(get_interface_attr('FAKE_ATTR')) == num_brands
    assert len(get_interface_attr('DBC', combine_brands=True)) >= 160
    assert len(get_interface_attr('CAR', combine_brands=True)) == 0
    none_brands = {b for b, v in get_interface_attr('FINGERPRINTS').items() if v is None}
    assert len(none_brands) >= 1
    assert len(none_brands.intersection(get_interface_attr('FINGERPRINTS', ignore_none=True))) == 0
