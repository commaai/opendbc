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
    # Format should be [breakpoints, gains] based on the constructor
    kp = [[0, 15, 30], [0.5, 1.0, 1.5]]  # [speeds, gains]
    ki = [[0, 15, 30], [0.1, 0.2, 0.3]]
    kd = [[0, 15, 30], [0.05, 0.1, 0.15]]
    
    controller = PIDController(kp, ki, k_f=0.0, k_d=kd, neg_limit=-10, pos_limit=10)
    
    # Test at low speed (should interpolate between first two gains)
    error = 2.0
    speed = 5.0
    output = controller.update(error, speed=speed)
    
    # P term should be interpolated: 0.5 + (1.0-0.5)*(5/15) = 0.5 + 0.5*0.333 = 0.667
    expected_kp = 0.5 + (1.0 - 0.5) * (5.0 / 15.0)
    assert abs(controller.p - expected_kp * error) < 0.01
    
  def test_override_mode(self):
    # Test override mode (covers override branch)
    controller = PIDController(1.0, 0.1, 0.05, neg_limit=-10, pos_limit=10)
    
    # Build up some integral
    for _ in range(10):
      controller.update(1.0, 0.0)
    
    initial_integral = controller.i
    
    # Update with override=True (should unwind integrator)
    controller.update(1.0, 0.0, override=True)
    
    # Integral should have moved towards zero
    assert abs(controller.i) < abs(initial_integral)
    
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
    # Test error_integral property getter
    controller = PIDController(1.0, 0.1, 0.05, neg_limit=-10, pos_limit=10)
    
    # Build up integral through updates
    for _ in range(5):
      controller.update(2.0, 0.0)
    
    # Check that error_integral returns i/k_i
    expected_error_integral = controller.i / controller.k_i
    assert abs(controller.error_integral - expected_error_integral) < 0.001
    
  def test_saturation_limits(self):
    # Test output saturation
    controller = PIDController(10.0, 0.0, 0.0, neg_limit=-5, pos_limit=5)
    
    # Large error should saturate
    output = controller.update(10.0, 0.0)
    assert output == 5.0  # Should hit positive limit
    
    output = controller.update(-10.0, 0.0)
    assert output == -5.0  # Should hit negative limit