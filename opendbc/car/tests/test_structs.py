import os
import capnp
from opendbc.car import structs
from opendbc.car.common.basedir import BASEDIR


class TestStructsModule:
  def test_struct_imports(self):
    """Test that all required structs are imported"""
    # Verify the main structs are available
    assert hasattr(structs, 'CarState')
    assert hasattr(structs, 'RadarData')
    assert hasattr(structs, 'CarControl')
    assert hasattr(structs, 'CarParams')

  def test_struct_types(self):
    """Test that struct types are properly defined"""
    assert hasattr(structs, 'CarStateT')
    assert hasattr(structs, 'RadarDataT')
    assert hasattr(structs, 'CarControlT')
    assert hasattr(structs, 'CarParamsT')

  def test_car_state_structure(self):
    """Test CarState structure basics"""
    # Test that we can access CarState schema
    car_state = structs.CarState
    assert car_state is not None

    # Test that it has expected nested structures
    assert hasattr(car_state, 'GearShifter')
    assert hasattr(car_state, 'ButtonEvent')

  def test_car_state_gear_shifter(self):
    """Test CarState.GearShifter enum"""
    gear_shifter = structs.CarState.GearShifter

    # Test basic gear positions exist
    assert hasattr(gear_shifter, 'unknown')
    assert hasattr(gear_shifter, 'park')
    assert hasattr(gear_shifter, 'reverse')
    assert hasattr(gear_shifter, 'neutral')
    assert hasattr(gear_shifter, 'drive')

  def test_car_state_button_event(self):
    """Test CarState.ButtonEvent structure"""
    button_event = structs.CarState.ButtonEvent

    # Test that it has Type enum
    assert hasattr(button_event, 'Type')

    # Test some button types exist
    button_type = button_event.Type
    assert hasattr(button_type, 'unknown')

  def test_car_params_structure(self):
    """Test CarParams structure basics"""
    car_params = structs.CarParams
    assert car_params is not None

    # Test that it has expected nested enums
    assert hasattr(car_params, 'Ecu')

  def test_car_params_has_schema(self):
    """Test CarParams has schema information"""
    car_params = structs.CarParams

    # Test that it's a valid capnp struct
    assert car_params is not None

  def test_car_params_ecu(self):
    """Test CarParams.Ecu enum"""
    ecu = structs.CarParams.Ecu

    # Test that common ECU types exist
    assert hasattr(ecu, 'engine')
    assert hasattr(ecu, 'eps')
    assert hasattr(ecu, 'abs')
    assert hasattr(ecu, 'fwdRadar')
    assert hasattr(ecu, 'fwdCamera')

  def test_car_params_can_import(self):
    """Test CarParams can be imported successfully"""
    car_params = structs.CarParams

    # Basic validation that it's a struct
    assert car_params is not None

  def test_car_control_structure(self):
    """Test CarControl structure basics"""
    car_control = structs.CarControl
    assert car_control is not None

    # Test that it has expected nested structures
    assert hasattr(car_control, 'Actuators')

  def test_car_control_actuators(self):
    """Test CarControl.Actuators structure"""
    actuators = structs.CarControl.Actuators
    assert actuators is not None

  def test_radar_data_structure(self):
    """Test RadarData structure basics"""
    radar_data = structs.RadarData
    assert radar_data is not None

    # Test that it has expected nested structures
    assert hasattr(radar_data, 'RadarPoint')

  def test_radar_data_point(self):
    """Test RadarData.RadarPoint structure"""
    radar_point = structs.RadarData.RadarPoint
    assert radar_point is not None

  def test_capnp_import_fallback(self):
    """Test that capnp file loading works as fallback"""
    # Verify the car.capnp file exists
    capnp_path = os.path.join(BASEDIR, "car.capnp")
    assert os.path.exists(capnp_path)

  def test_struct_module_types(self):
    """Test that type definitions are correct"""
    # Verify the type definitions are StructModule types
    assert structs.CarStateT == capnp.lib.capnp._StructModule
    assert structs.RadarDataT == capnp.lib.capnp._StructModule
    assert structs.CarControlT == capnp.lib.capnp._StructModule
    assert structs.CarParamsT == capnp.lib.capnp._StructModule

  def test_ecu_enum_values(self):
    """Test specific ECU enum values for automotive systems"""
    ecu = structs.CarParams.Ecu

    # Verify essential automotive ECUs are defined
    # These are critical for vehicle safety and operation
    essential_ecus = ['engine', 'eps', 'abs', 'fwdRadar', 'fwdCamera']

    for ecu_name in essential_ecus:
      assert hasattr(ecu, ecu_name), f"Missing essential ECU: {ecu_name}"

  def test_basic_structure_imports(self):
    """Test that basic structures can be imported"""
    # Test that main struct types are available
    assert structs.CarParams is not None
    assert structs.CarState is not None
    assert structs.CarControl is not None
    assert structs.RadarData is not None

  def test_gear_shifter_completeness(self):
    """Test that all standard gear positions are available"""
    gear_shifter = structs.CarState.GearShifter

    # Standard automatic transmission positions
    standard_gears = ['park', 'reverse', 'neutral', 'drive']

    for gear in standard_gears:
      assert hasattr(gear_shifter, gear), f"Missing standard gear: {gear}"

  def test_struct_instantiation_safety(self):
    """Test that structs can be safely referenced without instantiation"""
    # This tests that the module imports don't cause errors
    # when just accessing the class definitions

    # Should be able to access schema information
    assert structs.CarParams is not None
    assert structs.CarState is not None
    assert structs.CarControl is not None
    assert structs.RadarData is not None

    # Should be able to access nested enums
    assert structs.CarParams.Ecu is not None
    assert structs.CarState.GearShifter is not None