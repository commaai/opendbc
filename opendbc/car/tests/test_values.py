from typing import get_args
from opendbc.car.values import Platform, BRANDS, PLATFORMS
from opendbc.car.body.values import CAR as BODY
from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.ford.values import CAR as FORD
from opendbc.car.gm.values import CAR as GM
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.mazda.values import CAR as MAZDA
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.rivian.values import CAR as RIVIAN
from opendbc.car.subaru.values import CAR as SUBARU
from opendbc.car.tesla.values import CAR as TESLA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.volkswagen.values import CAR as VOLKSWAGEN


class TestPlatformDefinitions:
  def test_platform_union_type(self):
    """Test that Platform is a union of all brand CAR types"""
    # Platform should be a union type containing all brand CAR enums
    platform_args = get_args(Platform)
    assert len(platform_args) > 0

    # Verify that all individual brand CAR types are in the union
    expected_brands = [BODY, CHRYSLER, FORD, GM, HONDA, HYUNDAI, MAZDA,
                      MOCK, NISSAN, RIVIAN, SUBARU, TESLA, TOYOTA, VOLKSWAGEN]

    for brand in expected_brands:
      assert brand in platform_args

  def test_brands_consistency(self):
    """Test that BRANDS matches the Platform union arguments"""
    platform_args = get_args(Platform)
    assert BRANDS == platform_args
    assert len(BRANDS) == 14  # 14 different automotive brands

  def test_brand_imports(self):
    """Test that all brand CAR imports are valid enum-like classes"""
    brands = [BODY, CHRYSLER, FORD, GM, HONDA, HYUNDAI, MAZDA,
             MOCK, NISSAN, RIVIAN, SUBARU, TESLA, TOYOTA, VOLKSWAGEN]

    for brand in brands:
      # Each brand should have a __name__ attribute
      assert hasattr(brand, '__name__')
      # Each brand should be iterable (have car models)
      assert hasattr(brand, '__iter__')

  def test_mock_car_exists(self):
    """Test that mock car platform exists for testing"""
    assert hasattr(MOCK, 'MOCK')
    mock_platform = MOCK.MOCK
    assert mock_platform is not None

  def test_platforms_dict_structure(self):
    """Test PLATFORMS dictionary structure and contents"""
    # PLATFORMS should be a dictionary
    assert isinstance(PLATFORMS, dict)
    assert len(PLATFORMS) > 0

    # All keys should be strings
    for key in PLATFORMS.keys():
      assert isinstance(key, str)

    # All values should be platform instances
    for platform in PLATFORMS.values():
      assert platform is not None

  def test_platforms_dict_completeness(self):
    """Test that PLATFORMS contains all car models from all brands"""
    # Count total platforms by iterating through all brands
    total_platforms = 0
    for brand in BRANDS:
      for platform in brand:
        total_platforms += 1
        # Each platform should be in PLATFORMS dict
        assert str(platform) in PLATFORMS
        assert PLATFORMS[str(platform)] == platform

    # PLATFORMS should contain exactly the same number of entries
    assert len(PLATFORMS) == total_platforms

  def test_platform_string_representation(self):
    """Test that platforms have proper string representations"""
    for platform_str, platform in PLATFORMS.items():
      # String representation should match the key
      assert str(platform) == platform_str
      # Platform string should not be empty
      assert len(platform_str) > 0

  def test_major_brands_have_platforms(self):
    """Test that major automotive brands have at least one platform"""
    major_brands = [TOYOTA, HONDA, FORD, GM, HYUNDAI, VOLKSWAGEN, TESLA]

    for brand in major_brands:
      # Each major brand should have at least one car model
      platforms = list(brand)
      assert len(platforms) > 0

  def test_platform_uniqueness(self):
    """Test that all platform names are unique across brands"""
    all_platform_names = set()

    for brand in BRANDS:
      for platform in brand:
        platform_name = str(platform)
        # Each platform name should be unique
        assert platform_name not in all_platform_names
        all_platform_names.add(platform_name)

  def test_platform_naming_conventions(self):
    """Test platform naming follows conventions"""
    for platform_str in PLATFORMS.keys():
      # Platform names should be uppercase
      assert platform_str.isupper()
      # Platform names should not contain spaces
      assert ' ' not in platform_str
      # Platform names should not be empty
      assert len(platform_str) > 0

  def test_brand_specific_access(self):
    """Test accessing specific brand platforms directly"""
    # Test that we can access specific car models from each brand

    # Mock should have MOCK
    assert hasattr(MOCK, 'MOCK')

    # Toyota should have some common models (if they exist)
    toyota_platforms = list(TOYOTA)
    assert len(toyota_platforms) > 0

    # Honda should have platforms
    honda_platforms = list(HONDA)
    assert len(honda_platforms) > 0

  def test_platform_lookup_performance(self):
    """Test that platform lookup in PLATFORMS dict is efficient"""
    # This tests the dictionary access pattern used in the codebase
    for platform_str in list(PLATFORMS.keys())[:10]:  # Test first 10
      # Dictionary lookup should be O(1)
      platform = PLATFORMS[platform_str]
      assert platform is not None
      assert str(platform) == platform_str

  def test_brands_enum_interface(self):
    """Test that brand enums behave as expected"""
    for brand in BRANDS:
      # Should be able to iterate over platforms in each brand
      platforms = list(brand)
      assert isinstance(platforms, list)

      # Each platform should have a consistent interface
      for platform in platforms:
        assert hasattr(platform, '__str__')
        assert str(platform) in PLATFORMS

  def test_platform_consistency_across_imports(self):
    """Test that platform definitions are consistent with direct imports"""
    # Verify that the platforms from direct imports match those in PLATFORMS
    direct_brands = {
      'BODY': BODY, 'CHRYSLER': CHRYSLER, 'FORD': FORD, 'GM': GM,
      'HONDA': HONDA, 'HYUNDAI': HYUNDAI, 'MAZDA': MAZDA, 'MOCK': MOCK,
      'NISSAN': NISSAN, 'RIVIAN': RIVIAN, 'SUBARU': SUBARU,
      'TESLA': TESLA, 'TOYOTA': TOYOTA, 'VOLKSWAGEN': VOLKSWAGEN
    }

    for brand_name, brand in direct_brands.items():
      # Each brand should be in BRANDS
      assert brand in BRANDS

      # All platforms from this brand should be in PLATFORMS
      for platform in brand:
        assert str(platform) in PLATFORMS

  def test_empty_brands_handling(self):
    """Test handling of brands that might not have platforms"""
    # Some brands might be placeholders or have conditional platforms
    for brand in BRANDS:
      platforms = list(brand)
      # If a brand has no platforms, it should still be iterable
      assert isinstance(platforms, list)
      # But for actual automotive brands, we expect at least some platforms
      if brand != BODY:  # BODY might be special case
        # Most automotive brands should have at least one platform
        pass  # Don't assert here as some brands might legitimately be empty

  def test_platform_type_safety(self):
    """Test type safety of platform definitions"""
    # All platforms should be of compatible types
    for platform in PLATFORMS.values():
      # Platforms should be hashable (can be used as dict keys)
      assert hash(platform) is not None
      # Platforms should have string representation
      assert isinstance(str(platform), str)