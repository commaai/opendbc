TOLERANCE = 1e-3

CARSTATE_FIELDS = [
  "vEgo", "aEgo", "vEgoRaw", "yawRate", "standstill",
  "gasPressed", "brake", "brakePressed", "regenBraking", "parkingBrake", "brakeHoldActive",
  "steeringAngleDeg", "steeringAngleOffsetDeg", "steeringRateDeg", "steeringTorque", "steeringTorqueEps",
  "steeringPressed", "steerFaultTemporary", "steerFaultPermanent",
  "stockAeb", "stockFcw", "stockLkas", "espDisabled", "espActive", "accFaulted",
  "cruiseState.enabled", "cruiseState.available", "cruiseState.speed", "cruiseState.standstill",
  "cruiseState.nonAdaptive", "cruiseState.speedCluster",
  "gearShifter", "leftBlinker", "rightBlinker", "genericToggle",
  "doorOpen", "seatbeltUnlatched", "leftBlindspot", "rightBlindspot",
  "canValid", "canTimeout",
]


def get_value(obj, field):
  for p in field.split("."):
    obj = getattr(obj, p, None)
  return obj.raw if hasattr(obj, "raw") else obj


def differs(v1, v2):
  if isinstance(v1, float) and isinstance(v2, float):
    return abs(v1 - v2) > TOLERANCE
  return v1 != v2
