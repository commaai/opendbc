def create_solenoid_cmd(packer, left, right, start):
  values = {
    "SOLENOID_L": left,
    "SOLENOID_R": right,
    "SOLENOID_START": start,
  }
  return packer.make_can_msg("SOLENOID_CMD", 0, values)
