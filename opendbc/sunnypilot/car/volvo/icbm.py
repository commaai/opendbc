"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# Volvo ACC buttons change speed in 5 km/h steps (always to the next multiple
# of 5). Floor the planner target to the nearest 5 km/h multiple so the
# high-level controller stops pressing once cruise reaches that floor, instead
# of oscillating between two multiples (e.g. target=93 → floor=90 → hold at 90
# rather than bouncing between 90 and 95).
ACC_STEP_KMH = 5

BUTTONS = {
  SendButtonState.increase: {"set_plus": True},
  SendButtonState.decrease: {"minus": True},
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC_SP, CS, packer, frame, last_button_frame) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    send = self.ICBM.sendButton

    if send != SendButtonState.none:
      # vTarget is already in km/h (set by controller.py after MS_TO_KPH conversion)
      floor_target_kph = int(self.ICBM.vTarget // ACC_STEP_KMH) * ACC_STEP_KMH
      cruise_kph = round(CS.out.cruiseState.speedCluster * CV.MS_TO_KPH)

      if send == SendButtonState.increase and cruise_kph >= floor_target_kph:
        send = SendButtonState.none
      elif send == SendButtonState.decrease and cruise_kph <= floor_target_kph:
        send = SendButtonState.none

    if send != SendButtonState.none:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        can_sends.extend([volvocan.create_button_msg(packer, **BUTTONS[send])] * CarControllerParams.BUTTON_BURST)
        self.last_button_frame = self.frame

    return can_sends
