def test_test_route_present(subtests):
  from opendbc.car.values import PLATFORMS
  from opendbc.car.tests.routes import non_tested_cars, routes

  tested_platforms = [r.car_model for r in routes]
  tested_platforms_set = set(tested_platforms) | set(non_tested_cars)

  for platform in PLATFORMS.keys():
    with subtests.test(msg=f"test for {platform}"):
      assert platform in tested_platforms_set, f"Missing test route for {platform}. Add a route to opendbc/car/tests/routes.py"
