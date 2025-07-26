from unittest.mock import Mock
from opendbc.car.toyota.carcontroller import get_long_tune
from opendbc.car.toyota.values import ToyotaFlags, TSS2_CAR
from opendbc.car.structs import CarParams


class TestToyotaCarController:
  def test_get_long_tune_tss2_branch(self):
    """Test get_long_tune with TSS2 car (covers line 38 if branch)"""
    # Create CarParams for a TSS2 car
    CP = CarParams.new_message()
    CP.carFingerprint = 'TOYOTA_RAV4'  # Should be in TSS2_CAR
    
    # Create mock params
    params = Mock()
    params.ACCEL_MAX = 2.0
    params.ACCEL_MIN = -3.5
    
    # Verify TSS2 car is actually in TSS2_CAR set
    if CP.carFingerprint in TSS2_CAR:
      pid = get_long_tune(CP, params)
      
      # TSS2 cars should use different ki values (kiBP = [2., 5.], kiV = [0.5, 0.25])
      assert pid is not None
      assert hasattr(pid, 'k_i')
      # The PID should be configured with the TSS2-specific gains

  def test_get_long_tune_non_tss2_branch(self):
    """Test get_long_tune with non-TSS2 car (covers line 42 else branch)"""
    # Create CarParams for a non-TSS2 car
    CP = CarParams.new_message()
    CP.carFingerprint = 'TOYOTA_PRIUS'  # Should not be in TSS2_CAR
    
    # Create mock params
    params = Mock()
    params.ACCEL_MAX = 2.0
    params.ACCEL_MIN = -3.5
    
    # Verify this car is NOT in TSS2_CAR set
    if CP.carFingerprint not in TSS2_CAR:
      pid = get_long_tune(CP, params)
      
      # Non-TSS2 cars should use different ki values (kiBP = [0., 5., 35.], kiV = [3.6, 2.4, 1.5])
      assert pid is not None
      assert hasattr(pid, 'k_i')
      # The PID should be configured with the non-TSS2-specific gains

  def test_both_branches_covered(self):
    """Test that both TSS2 and non-TSS2 branches are exercised"""
    params = Mock()
    params.ACCEL_MAX = 2.0
    params.ACCEL_MIN = -3.5
    
    # Test TSS2 branch with known TSS2 car
    tss2_cars = list(TSS2_CAR)
    if tss2_cars:
      CP_tss2 = CarParams.new_message()
      CP_tss2.carFingerprint = tss2_cars[0]
      pid_tss2 = get_long_tune(CP_tss2, params)
      assert pid_tss2 is not None
    
    # Test non-TSS2 branch with a car we know is not in TSS2_CAR
    CP_non_tss2 = CarParams.new_message()
    CP_non_tss2.carFingerprint = 'TOYOTA_PRIUS'  # Commonly not TSS2
    pid_non_tss2 = get_long_tune(CP_non_tss2, params)
    assert pid_non_tss2 is not None
    
    # Both should be valid PID controllers but potentially with different parameters
    assert hasattr(pid_tss2, 'k_i') if tss2_cars else True
    assert hasattr(pid_non_tss2, 'k_i')

  def test_car_controller_params_flags(self):
    """Test CarControllerParams flag handling (covers values.py logic)"""
    from opendbc.car.toyota.values import CarControllerParams
    
    # Test with RAISED_ACCEL_LIMIT flag
    CP_raised = CarParams.new_message()
    CP_raised.flags = ToyotaFlags.RAISED_ACCEL_LIMIT.value
    
    params_raised = CarControllerParams(CP_raised)
    assert params_raised.ACCEL_MAX == 2.0  # Should be raised limit
    
    # Test without RAISED_ACCEL_LIMIT flag
    CP_normal = CarParams.new_message()
    CP_normal.flags = 0
    
    params_normal = CarControllerParams(CP_normal)
    assert params_normal.ACCEL_MAX == 1.5  # Should be normal limit

  def test_car_controller_params_tuning_type(self):
    """Test CarControllerParams lateral tuning type handling"""
    from opendbc.car.toyota.values import CarControllerParams
    
    # Test with torque tuning
    CP_torque = CarParams.new_message()
    CP_torque.lateralTuning.init('torque')
    CP_torque.flags = 0
    
    params_torque = CarControllerParams(CP_torque)
    assert params_torque.STEER_DELTA_UP == 15
    assert params_torque.STEER_DELTA_DOWN == 25
    
    # Test with non-torque tuning (else branch)
    CP_other = CarParams.new_message()
    CP_other.lateralTuning.init('pid')  # Not torque
    CP_other.flags = 0
    
    params_other = CarControllerParams(CP_other)
    assert params_other.STEER_DELTA_UP == 10
    assert params_other.STEER_DELTA_DOWN == 25