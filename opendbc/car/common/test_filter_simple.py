from opendbc.car.common.filter_simple import FirstOrderFilter


class TestFirstOrderFilter:
  def test_initialized_filter(self):
    # Test filter when initialized with a value
    dt = 0.01
    tau = 0.1
    initial_value = 10.0

    filter_obj = FirstOrderFilter(initial_value, dt, tau)

    # Should return the initial value on first call
    result = filter_obj.update(15.0)

    # Result should be between initial and new value due to filtering
    assert initial_value < result < 15.0

  def test_uninitialized_filter(self):
    # Test filter when not initialized (covers the initialized=False branch)
    dt = 0.01
    rc = 0.1

    filter_obj = FirstOrderFilter(10.0, rc, dt, initialized=False)

    # First update should return the input value directly and set initialized=True
    result = filter_obj.update(20.0)
    assert result == 20.0
    assert filter_obj.initialized is True

    # Second update should apply filtering
    result2 = filter_obj.update(25.0)
    assert 20.0 < result2 < 25.0

  def test_filter_convergence(self):
    # Test that filter converges to steady state
    dt = 0.01
    tau = 0.05  # Faster response

    filter_obj = FirstOrderFilter(0.0, dt, tau)

    # Apply same input multiple times
    target = 100.0
    for _ in range(100):
      result = filter_obj.update(target)

    # Should be very close to target after many iterations
    assert abs(result - target) < 0.1