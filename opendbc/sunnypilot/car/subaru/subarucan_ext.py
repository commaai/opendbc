from opendbc.car.subaru.values import CanBus, SubaruFlags


def create_counter(msg):
  return (msg["COUNTER"] + 1) % 0x10


def create_throttle(packer, CP, throttle_msg, send_resume):
  if CP.flags & SubaruFlags.PREGLOBAL:
    values = {s: throttle_msg[s] for s in [
      "Throttle_Pedal",
      "Signal1",
      "Not_Full_Throttle",
      "Signal2",
      "Engine_RPM",
      "Off_Throttle",
      "Signal3",
      "Throttle_Cruise",
      "Throttle_Combo",
      "Throttle_Body",
      "Off_Throttle_2",
      "Signal4",
    ]}
  else:
    values = {s: throttle_msg[s] for s in [
      "CHECKSUM",
      "Signal1",
      "Engine_RPM",
      "Signal2",
      "Throttle_Pedal",
      "Throttle_Cruise",
      "Throttle_Combo",
      "Signal3",
      "Off_Accel",
    ]}

  values["COUNTER"] = create_counter(throttle_msg)

  if send_resume:
    values["Throttle_Pedal"] = 5

  return packer.make_can_msg("Throttle", CanBus.camera, values)


def create_brake_pedal(packer, CP, brake_pedal_msg, send_resume):
  if CP.flags & SubaruFlags.PREGLOBAL:
    values = {s: brake_pedal_msg[s] for s in [
      "Speed",
      "Brake_Pedal",
      "Signal1",
    ]}
  else:
    values = {s: brake_pedal_msg[s] for s in [
      "CHECKSUM",
      "Signal1",
      "Speed",
      "Signal2",
      "Brake_Lights",
      "Signal3",
      "Brake_Pedal",
      "Signal4",
    ]}
    values["COUNTER"] = create_counter(brake_pedal_msg)

  if send_resume:
    values["Speed"] = 1 if CP.flags & SubaruFlags.PREGLOBAL else 3

  return packer.make_can_msg("Brake_Pedal", CanBus.camera, values)
