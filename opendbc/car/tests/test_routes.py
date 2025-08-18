from opendbc.car.values import PLATFORMS
from opendbc.car.tests.routes import non_tested_cars, routes


def test_test_route_present(subtests):
  tested_platforms = {r.car_model for r in routes}
  allowed_untested = set(non_tested_cars)

  for platform in sorted(PLATFORMS.keys()):
    with subtests.test(platform=platform):
      assert (
        platform in tested_platforms | allowed_untested
      ), f"Missing test route for {platform}. Add a route to opendbc/car/tests/routes.py"