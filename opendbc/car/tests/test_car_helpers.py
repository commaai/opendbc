from opendbc.car.car_helpers import _get_interface_names, FRAME_FINGERPRINT
from opendbc.car.structs import CarParams


class TestCarHelpers:
  def test_get_interface_names_basic(self):
    """Test _get_interface_names returns valid structure"""
    result = _get_interface_names()

    # Should return a dict
    assert isinstance(result, dict)
    # Should have some brands
    assert len(result) > 0
    # Each value should be a list
    for brand, models in result.items():
      assert isinstance(brand, str)
      assert isinstance(models, list)

  def test_frame_fingerprint_constant(self):
    """Test FRAME_FINGERPRINT constant is defined"""
    assert FRAME_FINGERPRINT == 100

  def test_car_params_creation(self):
    """Test basic CarParams creation works"""
    cp = CarParams.new_message()
    assert cp is not None

    # Test setting basic attributes
    cp.carFingerprint = 'TEST_CAR'
    assert cp.carFingerprint == 'TEST_CAR'