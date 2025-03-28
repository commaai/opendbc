import math

from cereal import car
from opendbc.can.parser import CANParser
from opendbc.car.interfaces import RadarInterfaceBase



class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.radar_off_can = CP.radarUnavailable

