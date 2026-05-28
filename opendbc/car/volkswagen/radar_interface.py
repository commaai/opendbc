from opendbc.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  # Volkswagen does not expose a radar interface to openpilot.
  # MEB is lateral-only; MQB/PQ/MLB have no radar interface either.
  pass
