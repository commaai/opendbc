import numpy as np
from opendbc.car.common.simple_kalman import KF1D, get_kalman_gain


class TestKF1D:
  def test_basic_filtering(self):
    # Test basic Kalman filter operation using proper matrix format
    # Setup like in interfaces.py
    A = [[1.0, 0.1], [0.0, 1.0]]
    C = [[1.0, 0.0]]
    Q = [[0.01, 0.0], [0.0, 0.01]]
    R = 0.1
    x0 = [[0.0], [0.0]]

    K = get_kalman_gain(0.01, np.array(A), np.array(C), np.array(Q), R)
    kf = KF1D(x0=x0, A=A, C=C[0], K=K)

    # Update with measurement
    result = kf.update(10.0)

    # State should move towards measurement
    assert 0 < result[0] < 10.0

  def test_set_x_method(self):
    # Test set_x method (covers the missing branch)
    A = [[1.0, 0.1], [0.0, 1.0]]
    C = [[1.0, 0.0]]
    Q = [[0.01, 0.0], [0.0, 0.01]]
    R = 0.1
    x0 = [[5.0], [0.0]]

    K = get_kalman_gain(0.01, np.array(A), np.array(C), np.array(Q), R)
    kf = KF1D(x0=x0, A=A, C=C[0], K=K)

    # Set new state value
    new_value = [[20.0], [1.0]]
    kf.set_x(new_value)

    # State should be updated
    assert kf.x[0][0] == 20.0
    assert kf.x[1][0] == 1.0

  def test_multiple_updates(self):
    # Test sequence of updates
    A = [[1.0, 0.1], [0.0, 1.0]]
    C = [[1.0, 0.0]]
    Q = [[0.01, 0.0], [0.0, 0.01]]
    R = 0.1
    x0 = [[0.0], [0.0]]

    K = get_kalman_gain(0.01, np.array(A), np.array(C), np.array(Q), R)
    kf = KF1D(x0=x0, A=A, C=C[0], K=K)

    measurements = [1.0, 2.0, 3.0, 4.0, 5.0]

    for measurement in measurements:
      result = kf.update(measurement)

    # Should converge towards the measurements (less strict bounds)
    assert result[0] > 0.5  # Should be moving in the right direction


class TestGetKalmanGain:
  def test_basic_gain_calculation(self):
    # Test basic Kalman gain calculation with proper parameters
    dt = 0.01
    A = np.array([[1.0, 0.1], [0.0, 1.0]])
    C = np.array([[1.0, 0.0]])
    Q = np.array([[0.01, 0.0], [0.0, 0.01]])
    R = 0.1

    gain = get_kalman_gain(dt, A, C, Q, R)

    # Gain should be a 2x1 matrix for this configuration
    assert gain.shape == (2, 1)
    assert np.all(gain >= 0)  # Gains should be non-negative

  def test_dt_scaling(self):
    # Test that gain scales with dt
    dt1 = 0.01
    dt2 = 0.02
    A = np.array([[1.0, 0.1], [0.0, 1.0]])
    C = np.array([[1.0, 0.0]])
    Q = np.array([[0.01, 0.0], [0.0, 0.01]])
    R = 0.1

    gain1 = get_kalman_gain(dt1, A, C, Q, R)
    gain2 = get_kalman_gain(dt2, A, C, Q, R)

    # Larger dt should generally give larger gain
    assert gain2[0, 0] > gain1[0, 0]

  def test_zero_dt(self):
    # Test edge case of zero dt
    A = np.array([[1.0, 0.1], [0.0, 1.0]])
    C = np.array([[1.0, 0.0]])
    Q = np.array([[0.01, 0.0], [0.0, 0.01]])
    R = 0.1

    gain = get_kalman_gain(0.0, A, C, Q, R)

    # Should handle gracefully and return reasonable gains
    assert gain.shape == (2, 1)
    assert np.all(gain >= 0)