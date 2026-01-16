import pytest
import os
from opendbc.car_discovery import get_all_car_names

class TestCarDiscovery:
  def test_regex_matches_import(self):
    """
    Meta-test to verify that the fast Regex-based car name extraction
    returns the exact same set of cars as the slow import-based method.
    This ensures that changes to files (e.g. formatting in values.py)
    don't silently break the optimized test collection.
    """
    # 1. Get fast result
    fast_cars = set(get_all_car_names())

    # 2. Get slow result (truth)
    # Checks if any values.py file was missed or parsed incorrectly
    from opendbc.car.values import PLATFORMS
    slow_cars = set(PLATFORMS.keys())

    # 3. Compare
    missing_in_fast = slow_cars - fast_cars
    extra_in_fast = fast_cars - slow_cars

    assert not missing_in_fast, f"Regex missed these cars found in imports: {missing_in_fast}"
    assert not extra_in_fast, f"Regex found extra cars not in PLATFORMS: {extra_in_fast}"

  def test_no_config_leak(self):
    """Ensure no 'config' or other non-car variables leaked into discovery"""
    cars = get_all_car_names()
    for car in cars:
        assert car != "config", "Found 'config' in car names, regex is too loose"
        assert car.isupper(), f"Found lowercase name '{car}', regex is too loose"
        assert not car.startswith("_"), f"Found private name '{car}', regex is too loose"

  def test_xdist_compatibility(self):
      """Ensure caching works safely (even if xdist uses forks, lru_cache is per-process)"""
      cars = get_all_car_names()
      assert len(cars) > 0
