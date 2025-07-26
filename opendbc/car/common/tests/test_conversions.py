import numpy as np
from opendbc.car.common.conversions import Conversions as CV


class TestConversions:
  def test_speed_conversions_consistency(self):
    """Test speed conversion constants are mathematically consistent"""
    # Test inverse relationships
    assert abs(CV.MPH_TO_KPH * CV.KPH_TO_MPH - 1.0) < 1e-10
    assert abs(CV.MS_TO_KPH * CV.KPH_TO_MS - 1.0) < 1e-10
    assert abs(CV.MS_TO_KNOTS * CV.KNOTS_TO_MS - 1.0) < 1e-10
  def test_compound_speed_conversions(self):
    """Test compound speed conversions are correct"""
    # MS_TO_MPH should equal MS_TO_KPH * KPH_TO_MPH
    assert abs(CV.MS_TO_MPH - (CV.MS_TO_KPH * CV.KPH_TO_MPH)) < 1e-10
    # MPH_TO_MS should equal MPH_TO_KPH * KPH_TO_MS
    assert abs(CV.MPH_TO_MS - (CV.MPH_TO_KPH * CV.KPH_TO_MS)) < 1e-10
  def test_angle_conversions_consistency(self):
    """Test angle conversion constants are mathematically consistent"""
    # Test inverse relationship
    assert abs(CV.DEG_TO_RAD * CV.RAD_TO_DEG - 1.0) < 1e-10
    # Test specific values
    assert abs(CV.DEG_TO_RAD - (np.pi / 180.)) < 1e-10
  def test_known_conversion_values(self):
    """Test conversions against known values"""
    # 1 mph = 1.609344 kph (exact)
    assert CV.MPH_TO_KPH == 1.609344
    # 1 m/s = 3.6 kph
    assert CV.MS_TO_KPH == 3.6
    # 1 lb = 0.453592 kg
    assert CV.LB_TO_KG == 0.453592
    # 1 m/s = 1.9438 knots (approximately)
    assert CV.MS_TO_KNOTS == 1.9438

  def test_practical_conversions(self):
    """Test practical conversion examples"""
    # 60 mph to kph
    speed_mph = 60
    speed_kph = speed_mph * CV.MPH_TO_KPH
    assert abs(speed_kph - 96.56064) < 0.00001
    # 100 kph to mph
    speed_kph = 100
    speed_mph = speed_kph * CV.KPH_TO_MPH
    assert abs(speed_mph - 62.137119) < 0.000001
    # 180 degrees to radians (pi)
    angle_deg = 180
    angle_rad = angle_deg * CV.DEG_TO_RAD
    assert abs(angle_rad - np.pi) < 1e-10
    # 100 pounds to kg
    weight_lb = 100
    weight_kg = weight_lb * CV.LB_TO_KG
    assert weight_kg == 45.3592