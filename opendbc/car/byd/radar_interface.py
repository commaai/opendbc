from opendbc.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = None
    self.pts = {}
    self.delay = 0

  def update(self, can_packets):
    return super().update(can_packets)
