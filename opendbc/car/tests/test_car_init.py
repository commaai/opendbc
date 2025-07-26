import pytest
from unittest.mock import Mock, patch
from opendbc.car import apply_hysteresis, clip, gen_empty_fingerprint
from opendbc.car.structs import CarParams


class TestCarInit:
  def test_apply_hysteresis_above_threshold(self):
    # Test hysteresis when value is above on_val
    result = apply_hysteresis(15.0, False, 10.0, 5.0)
    assert result is True
    
  def test_apply_hysteresis_below_threshold(self):
    # Test hysteresis when value is below off_val
    result = apply_hysteresis(3.0, True, 10.0, 5.0)
    assert result is False
    
  def test_apply_hysteresis_in_between(self):
    # Test hysteresis when value is between thresholds
    # Should maintain current state
    result = apply_hysteresis(7.0, True, 10.0, 5.0)
    assert result is True
    
    result = apply_hysteresis(7.0, False, 10.0, 5.0)
    assert result is False
    
  def test_clip_within_bounds(self):
    # Test clipping when value is within bounds
    result = clip(5.0, 0.0, 10.0)
    assert result == 5.0
    
  def test_clip_above_max(self):
    # Test clipping when value exceeds maximum
    result = clip(15.0, 0.0, 10.0)
    assert result == 10.0
    
  def test_clip_below_min(self):
    # Test clipping when value is below minimum
    result = clip(-5.0, 0.0, 10.0)
    assert result == 0.0
    
  def test_gen_empty_fingerprint_list(self):
    # Test generating empty fingerprint with list input
    addrs = [0x100, 0x200, 0x300]
    result = gen_empty_fingerprint(addrs)
    
    expected = {0x100: [0], 0x200: [0], 0x300: [0]}
    assert result == expected
    
  def test_gen_empty_fingerprint_dict(self):
    # Test generating empty fingerprint with dict input
    addrs = {0x100: 8, 0x200: 4}
    result = gen_empty_fingerprint(addrs)
    
    expected = {0x100: [0] * 8, 0x200: [0] * 4}
    assert result == expected