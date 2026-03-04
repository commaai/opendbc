"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.chrysler import chryslercan
from opendbc.car.chrysler.values import RAM_CARS
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CS, CC_SP, packer, frame, last_button_frame) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame
    ram_cars = self.CP.carFingerprint in RAM_CARS
    das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

    if self.ICBM.sendButton != SendButtonState.none:
      accel = self.ICBM.sendButton == SendButtonState.increase
      decel = self.ICBM.sendButton == SendButtonState.decrease

      if CS.button_counter != self.last_button_frame:
        self.last_button_frame = CS.button_counter

        if ram_cars:
          can_sends.append(chryslercan.create_cruise_buttons(packer, CS.button_counter, das_bus,
                                                             accel=accel, decel=decel))
        else:
          self.button_frame += 1
          button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
          if button_counter_offset is not None:
            can_sends.append(chryslercan.create_cruise_buttons(packer, CS.button_counter + button_counter_offset, das_bus,
                                                               accel=accel, decel=decel))

    return can_sends
