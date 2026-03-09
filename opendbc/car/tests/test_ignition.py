import unittest
from opendbc.car.ignition import get_ignition_can
from opendbc.car import structs

class MockParser:
  def __init__(self, vl):
    self.vl = vl

class TestIgnition(unittest.TestCase):
  def test_gm_ignition(self):
    CP = structs.CarParams.new_message()
    CP.carFingerprint = 'GM CHEVROLET BOLT EUV 2022'
    
    # Ignition ON (SystemPowerMode bit 1 set)
    pt_cp = MockParser({"ECMEngineStatus": {"SystemPowerMode": 0x2}})
    self.assertTrue(get_ignition_can(CP, pt_cp))
    
    # Ignition OFF
    pt_cp = MockParser({"ECMEngineStatus": {"SystemPowerMode": 0x0}})
    self.assertFalse(get_ignition_can(CP, pt_cp))

  def test_tesla_ignition(self):
    CP = structs.CarParams.new_message()
    CP.carFingerprint = 'TESLA MODEL 3'
    
    # Ignition ON (VEHICLE_POWER_STATE_DRIVE = 3)
    pt_cp = MockParser({"VCFRONT_vehiclePowerState": {"VCFRONT_vehiclePowerState": 0x3}})
    self.assertTrue(get_ignition_can(CP, pt_cp))
    
    # Ignition OFF
    pt_cp = MockParser({"VCFRONT_vehiclePowerState": {"VCFRONT_vehiclePowerState": 0x1}})
    self.assertFalse(get_ignition_can(CP, pt_cp))

  def test_non_trip_on_other_cars(self):
    # Test that a Toyota doesn't trip on GM signals
    CP = structs.CarParams.new_message()
    CP.carFingerprint = 'TOYOTA PRIUS'
    pt_cp = MockParser({"ECMEngineStatus": {"SystemPowerMode": 0x2}})
    self.assertFalse(get_ignition_can(CP, pt_cp))

if __name__ == "__main__":
  unittest.main()
