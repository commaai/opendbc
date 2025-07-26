import pytest
from opendbc.car.common.pid import PIDController


class TestPIDController:
  def test_number_gains(self):
    # Test with number gains (covers Number type branch)
    kp, ki, kd = 1.0, 0.1, 0.05
    controller = PIDController(kp, ki, kd, neg_limit=-10, pos_limit=10)
    
    # Test basic operation
    error = 5.0
    speed = 0.0
    output = controller.update(error, speed)
    
    # Should have proportional response
    assert output > 0
    assert controller.p == kp * error
    
  def test_list_gains_with_speed(self):
    # Test with list gains and speed interpolation
    kp = [0.5, 1.0, 1.5]  # Different gains at different speeds
    ki = [0.1, 0.2, 0.3]
    kd = [0.05, 0.1, 0.15]
    k_bp = [0, 15, 30]  # Speed breakpoints
    
    controller = PIDController(kp, ki, kd, k_bp, neg_limit=-10, pos_limit=10)
    
    # Test at low speed (should use first gains)
    error = 2.0
    speed = 5.0
    output = controller.update(error, speed)
    
    # P term should be between kp[0]*error and kp[1]*error
    expected_kp = 0.5 + (1.0 - 0.5) * (5.0 / 15.0)  # Linear interpolation
    assert abs(controller.p - expected_kp * error) < 0.01
    
  def test_override_mode(self):
    # Test override mode (covers override branch)
    controller = PIDController(1.0, 0.1, 0.05, neg_limit=-10, pos_limit=10)
    
    # First update to establish state
    controller.update(1.0, 0.0)
    
    # Enable override
    override_value = 7.5
    output = controller.update(2.0, 0.0, override=override_value)
    
    # Should return override value
    assert output == override_value
    
  def test_freeze_integrator(self):
    # Test freeze integrator functionality
    controller = PIDController(1.0, 0.5, 0.05, neg_limit=-10, pos_limit=10)
    
    # Build up some integral
    for _ in range(5):
      controller.update(1.0, 0.0)
    
    initial_integral = controller.i
    
    # Update with freeze_integrator=True
    controller.update(1.0, 0.0, freeze_integrator=True)
    
    # Integral should not change
    assert controller.i == initial_integral
    
  def test_error_integral_property(self):
    # Test error_integral property getter and setter
    controller = PIDController(1.0, 0.1, 0.05, neg_limit=-10, pos_limit=10)
    
    # Set error integral
    test_value = 5.0
    controller.error_integral = test_value
    
    # Should be retrievable
    assert controller.error_integral == test_value
    
    # Should affect integral term in next update
    controller.update(0.0, 0.0)  # Zero error, only integral term active
    assert controller.i == 0.1 * test_value  # ki * error_integral
    
  def test_saturation_limits(self):
    # Test output saturation
    controller = PIDController(10.0, 0.0, 0.0, neg_limit=-5, pos_limit=5)
    
    # Large error should saturate
    output = controller.update(10.0, 0.0)
    assert output == 5.0  # Should hit positive limit
    
    output = controller.update(-10.0, 0.0)
    assert output == -5.0  # Should hit negative limit