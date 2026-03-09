from opendbc.car import Bus, structs

def get_ignition_can(CP, pt_cp):
  """Detects ignition from CAN messages, moved from panda's can_common.h"""
  # GM
  if CP.carFingerprint.startswith('GM'):
    if "ECMEngineStatus" in pt_cp.vl:
      return (pt_cp.vl["ECMEngineStatus"]["SystemPowerMode"] & 0x2) != 0

  # Tesla Model 3/Y
  if CP.carFingerprint.startswith('TESLA'):
    if "VCFRONT_vehiclePowerState" in pt_cp.vl:
      return pt_cp.vl["VCFRONT_vehiclePowerState"]["VCFRONT_vehiclePowerState"] == 0x3

  # Mazda
  if CP.carFingerprint.startswith('MAZDA'):
    if "Engine_Data" in pt_cp.vl:
      return (pt_cp.vl["Engine_Data"]["IgnitionState"] >> 5) == 0x6

  return False
