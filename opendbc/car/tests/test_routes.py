import pytest


def get_platforms():
  from opendbc.car_discovery import get_all_car_names
  return get_all_car_names()


@pytest.mark.parametrize("platform", get_platforms())
def test_test_route_present(platform):
  from opendbc.car.tests.routes import non_tested_cars, routes

  tested_platforms = [r.car_model for r in routes]
  assert platform in set(tested_platforms) | set(non_tested_cars), \
    f"Missing test route for {platform}. Add a route to opendbc/car/tests/routes.py"
