import os
from unittest.mock import patch, mock_open
from opendbc.car.interfaces import (
  LatControlInputs, GEAR_SHIFTER_MAP, get_torque_params,
  V_CRUISE_MAX, MAX_CTRL_SPEED, ACCEL_MAX, ACCEL_MIN,
  ISO_LATERAL_ACCEL, ISO_LATERAL_JERK,
  TORQUE_PARAMS_PATH, TORQUE_OVERRIDE_PATH, TORQUE_SUBSTITUTE_PATH
)
from opendbc.car import structs


class TestInterfaces:
  def test_lat_control_inputs_namedtuple(self):
    """Test LatControlInputs NamedTuple creation and access"""
    inputs = LatControlInputs(
      lateral_acceleration=2.5,
      roll_compensation=0.1,
      vego=25.0,
      aego=1.5
    )

    assert inputs.lateral_acceleration == 2.5
    assert inputs.roll_compensation == 0.1
    assert inputs.vego == 25.0
    assert inputs.aego == 1.5

    # Test tuple-like access
    assert inputs[0] == 2.5
    assert inputs[1] == 0.1
    assert len(inputs) == 4

  def test_gear_shifter_map_completeness(self):
    """Test GEAR_SHIFTER_MAP contains expected mappings"""
    # Test basic gear mappings
    assert GEAR_SHIFTER_MAP['P'] == structs.CarState.GearShifter.park
    assert GEAR_SHIFTER_MAP['PARK'] == structs.CarState.GearShifter.park
    assert GEAR_SHIFTER_MAP['R'] == structs.CarState.GearShifter.reverse
    assert GEAR_SHIFTER_MAP['REVERSE'] == structs.CarState.GearShifter.reverse
    assert GEAR_SHIFTER_MAP['N'] == structs.CarState.GearShifter.neutral
    assert GEAR_SHIFTER_MAP['NEUTRAL'] == structs.CarState.GearShifter.neutral
    assert GEAR_SHIFTER_MAP['D'] == structs.CarState.GearShifter.drive
    assert GEAR_SHIFTER_MAP['DRIVE'] == structs.CarState.GearShifter.drive

    # Test special gear mappings
    assert GEAR_SHIFTER_MAP['E'] == structs.CarState.GearShifter.eco
    assert GEAR_SHIFTER_MAP['ECO'] == structs.CarState.GearShifter.eco
    assert GEAR_SHIFTER_MAP['S'] == structs.CarState.GearShifter.sport
    assert GEAR_SHIFTER_MAP['SPORT'] == structs.CarState.GearShifter.sport
    assert GEAR_SHIFTER_MAP['L'] == structs.CarState.GearShifter.low
    assert GEAR_SHIFTER_MAP['LOW'] == structs.CarState.GearShifter.low
    assert GEAR_SHIFTER_MAP['B'] == structs.CarState.GearShifter.brake
    assert GEAR_SHIFTER_MAP['BRAKE'] == structs.CarState.GearShifter.brake
    assert GEAR_SHIFTER_MAP['T'] == structs.CarState.GearShifter.manumatic
    assert GEAR_SHIFTER_MAP['MANUAL'] == structs.CarState.GearShifter.manumatic

  def test_constants_defined(self):
    """Test that important constants are properly defined"""
    assert V_CRUISE_MAX == 145
    assert MAX_CTRL_SPEED > 40  # Should be reasonable speed in m/s (~149 km/h)
    assert ACCEL_MAX == 2.0
    assert ACCEL_MIN == -3.5
    assert ACCEL_MIN < 0 < ACCEL_MAX  # Sensible acceleration range

    # ISO 11270 constants
    assert ISO_LATERAL_ACCEL == 3.0
    assert ISO_LATERAL_JERK == 5.0

  def test_torque_file_paths_defined(self):
    """Test torque configuration file paths are defined"""
    assert TORQUE_PARAMS_PATH.endswith('torque_data/params.toml')
    assert TORQUE_OVERRIDE_PATH.endswith('torque_data/override.toml')
    assert TORQUE_SUBSTITUTE_PATH.endswith('torque_data/substitute.toml')

    # Paths should be absolute
    assert os.path.isabs(TORQUE_PARAMS_PATH)
    assert os.path.isabs(TORQUE_OVERRIDE_PATH)
    assert os.path.isabs(TORQUE_SUBSTITUTE_PATH)

  def test_get_torque_params_substitution(self):
    """Test get_torque_params substitution logic"""
    # Mock TOML file contents
    substitute_data = {
      'HONDA_CIVIC': 'HONDA_ACCORD',
      'legend': ['param1', 'param2']
    }

    params_data = {
      'HONDA_ACCORD': [1.0, 2.0],
      'TOYOTA_PRIUS': [3.0, 4.0],
      'legend': ['param1', 'param2']
    }

    override_data = {
      'FORD_FOCUS': [5.0, 6.0],
      'legend': ['param1', 'param2']
    }

    with patch('builtins.open', mock_open()) as mock_file:
      with patch('tomllib.load') as mock_tomllib:
        # Return different data for each file
        mock_tomllib.side_effect = [substitute_data, params_data, override_data]

        # Clear cache to force reload
        get_torque_params.cache_clear()

        result = get_torque_params()

        # Should have params for all defined cars
        assert 'HONDA_ACCORD' in result
        assert 'HONDA_CIVIC' in result  # Should be substituted
        assert 'TOYOTA_PRIUS' in result
        assert 'FORD_FOCUS' in result

        # HONDA_CIVIC should have same params as HONDA_ACCORD due to substitution
        assert result['HONDA_CIVIC'] == result['HONDA_ACCORD']

        # Check parameter structure
        assert result['HONDA_ACCORD']['param1'] == 1.0
        assert result['HONDA_ACCORD']['param2'] == 2.0

        # Override should take precedence
        assert result['FORD_FOCUS']['param1'] == 5.0
        assert result['FORD_FOCUS']['param2'] == 6.0

  def test_get_torque_params_duplicate_definition_error(self):
    """Test get_torque_params raises error for duplicate definitions"""
    # Mock data with duplicate definition
    substitute_data = {'legend': ['param1', 'param2']}

    params_data = {
      'HONDA_ACCORD': [1.0, 2.0],
      'legend': ['param1', 'param2']
    }

    override_data = {
      'HONDA_ACCORD': [5.0, 6.0],  # Duplicate definition
      'legend': ['param1', 'param2']
    }

    with patch('builtins.open', mock_open()):
      with patch('tomllib.load') as mock_tomllib:
        mock_tomllib.side_effect = [substitute_data, params_data, override_data]

        get_torque_params.cache_clear()

        try:
          get_torque_params()
          assert False, "Should have raised RuntimeError"
        except RuntimeError as e:
          assert "is defined twice" in str(e)

  def test_get_torque_params_missing_candidate_error(self):
    """Test get_torque_params raises error for missing candidate"""
    substitute_data = {
      'HONDA_CIVIC': 'NONEXISTENT_CAR',
      'legend': ['param1', 'param2']
    }

    params_data = {'legend': ['param1', 'param2']}
    override_data = {'legend': ['param1', 'param2']}

    with patch('builtins.open', mock_open()):
      with patch('tomllib.load') as mock_tomllib:
        mock_tomllib.side_effect = [substitute_data, params_data, override_data]

        get_torque_params.cache_clear()

        try:
          get_torque_params()
          assert False, "Should have raised NotImplementedError"
        except NotImplementedError as e:
          assert "Did not find torque params for" in str(e)

  def test_get_torque_params_caching(self):
    """Test that get_torque_params uses caching"""
    with patch('builtins.open', mock_open()) as mock_file:
      with patch('tomllib.load') as mock_tomllib:
        mock_tomllib.side_effect = [
          {'legend': ['param1']},  # substitute
          {'CAR1': [1.0], 'legend': ['param1']},  # params
          {'legend': ['param1']}   # override
        ]

        get_torque_params.cache_clear()

        # First call
        result1 = get_torque_params()

        # Second call should use cache (no additional file reads)
        result2 = get_torque_params()

        # Results should be identical
        assert result1 == result2

        # Should have only called tomllib.load 3 times (once for each file)
        assert mock_tomllib.call_count == 3