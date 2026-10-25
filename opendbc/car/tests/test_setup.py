import json
import unittest

from opendbc.car.docs import get_all_car_docs
from opendbc.car.docs_definitions import CarHarness, SupportType
from opendbc.car.setup import get_setup_guide, get_setup_guides


class TestSetupGuides(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.cars = get_all_car_docs()
    cls.guides = get_setup_guides(cls.cars)

  def test_selector_coverage(self):
    expected = {c.name for c in self.cars if c.support_type == SupportType.UPSTREAM and c.name != "comma body"}
    self.assertEqual(set(self.guides) - {"default"}, expected)
    json.dumps(self.guides)
    for name, guide in self.guides.items():
      with self.subTest(car=name):
        self.assertTrue(guide["parts"])
        self.assertTrue(guide["sources"])
        self.assertEqual(len({s["id"] for s in guide["steps"]}), len(guide["steps"]))
        for step in guide["steps"]:
          self.assertTrue(step["body"])
          self.assertTrue(step["sources"])

  def test_remote_harnesses_never_show_camera_install(self):
    remote = {CarHarness.vw_j533, CarHarness.vw_meb, CarHarness.ford_q4, CarHarness.nissan_a,
              CarHarness.nissan_b, CarHarness.tesla_a, CarHarness.tesla_b, CarHarness.rivian_a, CarHarness.gmsdgm}
    for car in self.cars:
      if car.name in self.guides and any(p in remote for p in car.car_parts.parts):
        with self.subTest(car=car.name):
          guide = self.guides[car.name]
          self.assertFalse(any(s["media"].startswith("camera-") for s in guide["steps"]))
          self.assertIn("long OBD-C cable", " ".join(guide["steps"][3]["body"]))
          self.assertEqual(guide["steps"][3]["media"], "device-remote")

  def test_model_locations(self):
    cases = {"Ford F-150 2021-23": "footwell", "Ford Mustang Mach-E 2021-24": "cargo",
             "Ford Ranger 2024": "cluster", "Nissan Leaf 2018-23": "glovebox",
             "Nissan Rogue 2018-20": "cargo", "Volkswagen Caravelle 2020": "cluster",
             "Volkswagen Golf 2015-20": "footwell"}
    for name, media in cases.items():
      with self.subTest(car=name):
        self.assertEqual(self.guides[name]["steps"][0]["media"], media)

  def test_required_power_is_a_step(self):
    for car in self.cars:
      if car.name in self.guides and car.requires_comma_power:
        guide = self.guides[car.name]
        self.assertEqual(guide["power"], "required")
        self.assertIn("power", [s["id"] for s in guide["steps"]])

  def test_default_and_independent_content(self):
    default = get_setup_guide()
    self.assertEqual(len(default["steps"]), 5)
    default["steps"][0]["body"].append("changed")
    self.assertNotIn("changed", get_setup_guide()["steps"][0]["body"])
