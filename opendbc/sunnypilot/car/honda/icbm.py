"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CruiseButtons
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: CruiseButtons.RES_ACCEL,
  SendButtonState.decrease: CruiseButtons.DECEL_SET,
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC_SP, packer, frame, last_button_frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      send_button = BUTTONS[self.ICBM.sendButton]

      if (self.frame - self.last_button_frame) * DT_CTRL > 0.05:
        can_sends.append(hondacan.spam_buttons_command(packer, CAN, send_button, self.CP.carFingerprint))
        self.last_button_frame = self.frame

    return can_sends
