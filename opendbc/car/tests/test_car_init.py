import pytest
import numpy as np
from unittest.mock import Mock
from opendbc.car import (apply_hysteresis, gen_empty_fingerprint, create_button_events, 
                        apply_center_deadzone, rate_limit, common_fault_avoidance, ButtonType)
from opendbc.car.structs import CarParams


class TestCarInit:
  def test_apply_hysteresis_above_gap(self):
    # Test hysteresis when value exceeds steady + gap
    result = apply_hysteresis(15.0, 10.0, 2.0)
    assert result == 13.0  # val - hyst_gap
    
  def test_apply_hysteresis_below_gap(self):
    # Test hysteresis when value is below steady - gap
    result = apply_hysteresis(5.0, 10.0, 2.0)
    assert result == 7.0  # val + hyst_gap
    
  def test_apply_hysteresis_within_gap(self):
    # Test hysteresis when value is within gap
    # Should maintain current val_steady
    result = apply_hysteresis(9.0, 10.0, 2.0)
    assert result == 10.0  # unchanged val_steady
    
  def test_gen_empty_fingerprint(self):
    # Test generating empty fingerprint (no parameters)
    result = gen_empty_fingerprint()
    
    # Should return dict with 8 buses, each empty
    expected = {i: {} for i in range(8)}
    assert result == expected
    
  def test_create_button_events_no_change(self):
    # Test button events when no change
    buttons_dict = {1: ButtonType.accelCruise}
    result = create_button_events(1, 1, buttons_dict)
    
    # No events when buttons haven't changed
    assert result == []
    
  def test_create_button_events_with_change(self):
    # Test button events when buttons change
    buttons_dict = {
      0: ButtonType.unknown,
      1: ButtonType.accelCruise
    }
    result = create_button_events(1, 0, buttons_dict, unpressed_btn=0)
    
    # Should have one event for button press
    assert len(result) == 1
    assert result[0].pressed is True
    assert result[0].type == ButtonType.accelCruise
    
  def test_apply_center_deadzone_within(self):
    # Test deadzone when error is within deadzone
    result = apply_center_deadzone(0.5, 1.0)
    assert result == 0.0
    
  def test_apply_center_deadzone_outside(self):
    # Test deadzone when error is outside deadzone
    result = apply_center_deadzone(2.0, 1.0)
    assert result == 2.0
    
  def test_rate_limit(self):
    # Test rate limiting
    result = rate_limit(10.0, 5.0, -2.0, 3.0)
    # Should be clipped to last_value + up_step = 5.0 + 3.0 = 8.0
    assert result == 8.0
    
  def test_common_fault_avoidance_normal(self):
    # Test fault avoidance in normal conditions
    above_limit_frames, request = common_fault_avoidance(
      fault_condition=False, request=True, above_limit_frames=0,
      max_above_limit_frames=5, max_mismatching_frames=1
    )
    
    # Should reset counter and maintain request
    assert above_limit_frames == 0
    assert request is True
    
  def test_common_fault_avoidance_fault(self):
    # Test fault avoidance when fault condition is true
    above_limit_frames, request = common_fault_avoidance(
      fault_condition=True, request=True, above_limit_frames=6,
      max_above_limit_frames=5, max_mismatching_frames=1
    )
    
    # Should cut request when above limit
    assert request is False

  def test_apply_driver_steer_torque_limits_positive_last(self):
    # Test driver steer torque limits with positive last torque (covers line 117)
    from opendbc.car import apply_driver_steer_torque_limits
    from unittest.mock import Mock
    
    # Mock the LIMITS object
    LIMITS = Mock()
    LIMITS.STEER_MAX = 1000
    LIMITS.STEER_DRIVER_ALLOWANCE = 50
    LIMITS.STEER_DRIVER_FACTOR = 1.0
    LIMITS.STEER_DRIVER_MULTIPLIER = 1.0
    LIMITS.STEER_DELTA_DOWN = 100
    LIMITS.STEER_DELTA_UP = 80
    
    # Test with positive apply_torque_last to trigger line 117 branch
    result = apply_driver_steer_torque_limits(
      apply_torque=200,
      apply_torque_last=150,  # Positive value
      driver_torque=0,
      LIMITS=LIMITS
    )
    
    # Should apply rate limiting for positive last torque
    assert result <= 230  # 150 + 80 (STEER_DELTA_UP)

  def test_apply_driver_steer_torque_limits_negative_last(self):
    # Test driver steer torque limits with non-positive last torque
    from opendbc.car import apply_driver_steer_torque_limits
    from unittest.mock import Mock
    
    LIMITS = Mock()
    LIMITS.STEER_MAX = 1000
    LIMITS.STEER_DRIVER_ALLOWANCE = 50
    LIMITS.STEER_DRIVER_FACTOR = 1.0
    LIMITS.STEER_DRIVER_MULTIPLIER = 1.0
    LIMITS.STEER_DELTA_DOWN = 100
    LIMITS.STEER_DELTA_UP = 80
    
    # Test with non-positive apply_torque_last (else branch)
    result = apply_driver_steer_torque_limits(
      apply_torque=200,
      apply_torque_last=-50,  # Non-positive value
      driver_torque=0,
      LIMITS=LIMITS
    )
    
    # Should apply different rate limiting for non-positive last torque
    assert result >= -130  # -50 - 80 (STEER_DELTA_UP)

  def test_apply_dist_to_meas_limits_positive_last(self):
    # Test distance to measurement limits with positive last value (covers line 137)
    from opendbc.car import apply_dist_to_meas_limits
    
    result = apply_dist_to_meas_limits(
      val=100,
      val_last=50,  # Positive value
      val_meas=75,
      STEER_DELTA_UP=30,
      STEER_DELTA_DOWN=40,
      STEER_ERROR_MAX=25,
      STEER_MAX=200
    )
    
    # Should apply rate limiting for positive last value
    assert 50 <= result <= 100  # Should be rate limited

  def test_apply_dist_to_meas_limits_negative_last(self):
    # Test distance to measurement limits with non-positive last value
    from opendbc.car import apply_dist_to_meas_limits
    
    result = apply_dist_to_meas_limits(
      val=50,
      val_last=-30,  # Non-positive value
      val_meas=25,
      STEER_DELTA_UP=20,
      STEER_DELTA_DOWN=35,
      STEER_ERROR_MAX=15,
      STEER_MAX=100
    )
    
    # Should apply different rate limiting for non-positive last value
    assert result <= 40  # Should be limited differently