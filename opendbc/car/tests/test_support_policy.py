import unittest

from opendbc.car.support_policy import SupportedVehicle, get_platform_whitelist, get_vin_model_year, is_platform_vin_supported, is_vehicle_supported
from opendbc.car.toyota.values import CAR


def vin_for_year_code(code: str) -> str:
  return f"JTDBR32E0{code}1234567"


class TestSupportPolicy(unittest.TestCase):
  def test_vin_year(self):
    self.assertEqual(get_vin_model_year(vin_for_year_code("S")), 2025)
    self.assertEqual(get_vin_model_year(vin_for_year_code("T")), 2026)

  def test_whitelist_is_make_model_year_tuples(self):
    whitelist = get_platform_whitelist(CAR.TOYOTA_COROLLA_TSS2)
    self.assertIn(SupportedVehicle("Toyota", "Corolla", 2022), whitelist)

  def test_new_model_year_rejected_after_same_platform_match(self):
    self.assertTrue(is_platform_vin_supported(CAR.TOYOTA_COROLLA_TSS2, vin_for_year_code("N")))
    self.assertFalse(is_platform_vin_supported(CAR.TOYOTA_COROLLA_TSS2, vin_for_year_code("T")))

  def test_strict_tuple_check_does_not_confuse_models_in_one_manifest(self):
    whitelist = get_platform_whitelist(CAR.TOYOTA_COROLLA_TSS2)
    self.assertTrue(is_vehicle_supported(SupportedVehicle("Toyota", "Corolla Cross (Non-US only)", 2023), whitelist))
    self.assertFalse(is_vehicle_supported(SupportedVehicle("Toyota", "Corolla", 2023), whitelist))

  def test_unknown_vin_does_not_change_existing_behavior(self):
    self.assertTrue(is_platform_vin_supported(CAR.TOYOTA_COROLLA_TSS2, "0" * 17))
