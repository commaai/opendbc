from opendbc.car import structs
from opendbc.car.honda.values import HONDA_BOSCH_RADARLESS


class MadsCarController:
  def __init__(self):
    super().__init__()
    self.dashed_lanes = False

  def update(self, CP: structs.CarParams, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    enable_mads = CC_SP.mads.available

    if enable_mads:
      self.dashed_lanes = CC_SP.mads.enabled and not CC.latActive
    else:
      self.dashed_lanes = CC.hudControl.lanesVisible if CP.carFingerprint in HONDA_BOSCH_RADARLESS else False
