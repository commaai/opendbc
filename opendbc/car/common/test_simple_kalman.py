import pytest
from opendbc.car.common.simple_kalman import KF1D, get_kalman_gain


class TestKF1D:
  def test_basic_filtering(self):
    # Test basic Kalman filter operation
    kf = KF1D(x0=0.0, A=1.0, C=1.0, K=0.1)
    
    # Update with measurement
    kf.update(10.0)
    
    # State should move towards measurement
    assert 0 < kf.x < 10.0
    
  def test_set_x_method(self):
    # Test set_x method (covers the missing branch)
    kf = KF1D(x0=5.0, A=1.0, C=1.0, K=0.1)
    
    # Set new state value
    new_value = 20.0
    kf.set_x(new_value)
    
    # State should be updated
    assert kf.x == new_value
    
  def test_prediction_step(self):
    # Test prediction without measurement
    kf = KF1D(x0=10.0, A=1.1, C=1.0, K=0.1)  # A > 1 for growth
    
    initial_x = kf.x
    kf.predict()
    
    # State should change according to A matrix
    assert kf.x == initial_x * 1.1
    
  def test_multiple_updates(self):
    # Test sequence of updates
    kf = KF1D(x0=0.0, A=1.0, C=1.0, K=0.5)
    
    measurements = [1.0, 2.0, 3.0, 4.0, 5.0]
    
    for measurement in measurements:
      kf.update(measurement)
    
    # Should converge towards the measurements
    assert 4.0 < kf.x < 5.0


class TestGetKalmanGain:
  def test_basic_gain_calculation(self):
    # Test basic Kalman gain calculation
    dt = 0.01
    gain = get_kalman_gain(dt)
    
    # Gain should be reasonable for vehicle dynamics
    assert 0 < gain < 1.0
    
  def test_dt_scaling(self):
    # Test that gain scales with dt
    dt1 = 0.01
    dt2 = 0.02
    
    gain1 = get_kalman_gain(dt1)
    gain2 = get_kalman_gain(dt2)
    
    # Larger dt should generally give larger gain
    assert gain2 > gain1
    
  def test_zero_dt(self):
    # Test edge case of zero dt
    gain = get_kalman_gain(0.0)
    
    # Should handle gracefully
    assert gain >= 0