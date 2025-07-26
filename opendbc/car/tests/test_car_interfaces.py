import os
import unittest
import math
import hypothesis.strategies as st
from hypothesis import Phase, given, settings
from parameterized import parameterized
from collections.abc import Callable
from typing import Any

from opendbc.car import DT_CTRL, CanData, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.fingerprints import FW_VERSIONS
from opendbc.car.fw_versions import FW_QUERY_CONFIGS
from opendbc.car.interfaces import get_interface_attr
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.values import PLATFORMS

DrawType = Callable[[st.SearchStrategy], Any]

ALL_ECUS = {ecu for ecus in FW_VERSIONS.values() for ecu in ecus.keys()}
ALL_ECUS |= {ecu for config in FW_QUERY_CONFIGS.values() for ecu in config.extra_ecus}

ALL_REQUESTS = {tuple(r.request) for config in FW_QUERY_CONFIGS.values() for r in config.requests}

# From panda/python/__init__.py
DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

MAX_EXAMPLES = int(os.environ.get('MAX_EXAMPLES', '15'))


def get_fuzzy_car_interface_args(draw: DrawType) -> dict:
    fingerprint_strategy = st.fixed_dictionaries({
        0: st.dictionaries(
            st.integers(min_value=0, max_value=0x800),
            st.sampled_from(DLC_TO_LEN)
        )
    })
    car_fw_strategy = st.lists(st.builds(
        lambda fw, req: structs.CarParams.CarFw(
            ecu=fw[0], address=fw[1], subAddress=fw[2] or 0, request=req
        ),
        st.sampled_from(sorted(ALL_ECUS)),
        st.sampled_from(sorted(ALL_REQUESTS)),
    ))
    params_strategy = st.fixed_dictionaries({
        'fingerprints': fingerprint_strategy,
        'car_fw': car_fw_strategy,
        'alpha_long': st.booleans(),
    })

    params: dict = draw(params_strategy)
    params['fingerprints'] |= {
        key + 1: params['fingerprints'][0] for key in range(6)
    }
    return params


class TestCarInterfaces(unittest.TestCase):

    @parameterized.expand([(cn,) for cn in sorted(PLATFORMS)])
    @settings(max_examples=MAX_EXAMPLES, deadline=None,
              phases=(Phase.reuse, Phase.generate, Phase.shrink))
    @given(data=st.data())
    def test_car_interfaces(self, car_name, data):
        """Fuzz-test a single CarInterface implementation."""
        CarInterface = interfaces[car_name]
        args = get_fuzzy_car_interface_args(data.draw)

        car_params = CarInterface.get_params(
            car_name,
            args['fingerprints'],
            args['car_fw'],
            alpha_long=args['alpha_long'],
            is_release=False,
            docs=False
        )
        car_interface = CarInterface(car_params)

        # Basic sanity
        self.assertTrue(car_params)
        self.assertTrue(car_interface)

        # Mass & geometry
        self.assertGreater(car_params.mass, 1)
        self.assertGreater(car_params.wheelbase, 0)
        self.assertGreater(car_params.centerToFront,
                           car_params.wheelbase * 0.3)
        self.assertLess(car_params.centerToFront,
                        car_params.wheelbase * 0.7)
        self.assertGreater(car_params.maxLateralAccel, 0)

        # Longitudinal tuning
        lt = car_params.longitudinalTuning
        self.assertEqual(len(lt.kpV), len(lt.kpBP))
        self.assertEqual(len(lt.kiV), len(lt.kiBP))

        # Lateral tuning
        if car_params.steerControlType != structs.CarParams.SteerControlType.angle:
            tune = car_params.lateralTuning
            kind = tune.which()
            if kind == 'pid':
                if car_name != MOCK.MOCK:
                    self.assertFalse(math.isnan(tune.pid.kf))
                    self.assertGreater(tune.pid.kf, 0)
                    self.assertGreater(len(tune.pid.kpV), 0)
                    self.assertEqual(len(tune.pid.kpV), len(tune.pid.kpBP))
                    self.assertGreater(len(tune.pid.kiV), 0)
                    self.assertEqual(len(tune.pid.kiV), len(tune.pid.kiBP))
            elif kind == 'torque':
                self.assertFalse(math.isnan(tune.torque.kf))
                self.assertGreater(tune.torque.kf, 0)
                self.assertFalse(math.isnan(tune.torque.friction))
                self.assertGreater(tune.torque.friction, 0)

        # Drive the interface through a few cycles
        now_nanos = 0
        CC_reader = structs.CarControl().as_reader()
        for _ in range(10):
            car_interface.update([])
            car_interface.apply(CC_reader, now_nanos)
            now_nanos += DT_CTRL * 1e9

        CC2 = structs.CarControl()
        CC2.enabled = True
        CC2.latActive = True
        CC2.longActive = True
        CC2_reader = CC2.as_reader()
        for _ in range(10):
            car_interface.update([])
            car_interface.apply(CC2_reader, now_nanos)
            now_nanos += DT_CTRL * 1e9

        # Radar interface
        radar = CarInterface.RadarInterface(car_params)
        self.assertTrue(radar)
        radar.update([])
        if (not car_params.radarUnavailable and
                radar.rcp is not None and
                hasattr(radar, '_update') and
                hasattr(radar, 'trigger_msg')):
            radar._update([radar.trigger_msg])
        if (not car_params.radarUnavailable and radar.rcp is not None):
            cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
            rr = radar.update(cans)
            self.assertTrue(rr is None or len(rr.errors) > 0)

    @parameterized.expand([
        ("num_brands",),
    ])
    def test_interface_attrs(self, _):
        """Asserts basic behavior of interface attribute getter"""
        num_brands = len(get_interface_attr('CAR'))
        self.assertGreaterEqual(num_brands, 12)

        ret = get_interface_attr('FAKE_ATTR')
        self.assertEqual(len(ret), num_brands)

        combined = get_interface_attr('DBC', combine_brands=True)
        self.assertGreaterEqual(len(combined), 160)

        non_dict = get_interface_attr('CAR', combine_brands=True)
        self.assertEqual(len(non_dict), 0)

        none_brands = {b for b, v in get_interface_attr('FINGERPRINTS').items() if v is None}
        self.assertGreaterEqual(len(none_brands), 1)
        filtered = get_interface_attr('FINGERPRINTS', ignore_none=True)
        self.assertTrue(none_brands.isdisjoint(filtered),
                        f'Brands with None values still present: {none_brands & set(filtered)}')


if __name__ == "__main__":
    unittest.main()
