import pytest


def get_platforms():
  from opendbc.car_discovery import get_all_car_names
  return [(n, n) for n in get_all_car_names()]


class TestPlatformConfigs:
  @pytest.mark.parametrize("name, platform_name", get_platforms())
  def test_configs(self, name, platform_name):
    from opendbc.car.values import PLATFORMS
    platform = PLATFORMS[platform_name]
    assert platform.config._frozen

    if platform != "MOCK":
      assert len(platform.config.dbc_dict) > 0
    assert len(platform.config.platform_str) > 0

    assert name == platform.config.platform_str

    assert platform.config.specs is not None
