"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum
from collections import namedtuple

from opendbc.car import Bus, structs
from opendbc.car.chrysler.values import RAM_CARS

from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser

MadsDataSP = namedtuple("MadsDataSP",
                        ["enable_mads", "paused", "lkas_disabled"])

ButtonType = structs.CarState.ButtonEvent.Type


class MadsCarController:
  def __init__(self):
    self.mads = MadsDataSP(False, False, False)

  @staticmethod
  def create_lkas_heartbit(packer, lkas_heartbit, mads):
    # LKAS_HEARTBIT (0x2D9) LKAS heartbeat
    values = {s: lkas_heartbit[s] for s in [
      "LKAS_DISABLED",
      "AUTO_HIGH_BEAM",
      "FORWARD_1",
      "FORWARD_2",
      "FORWARD_3",
    ]}

    if mads.enable_mads:
      values["LKAS_DISABLED"] = 1 if mads.lkas_disabled else 0

    return packer.make_can_msg("LKAS_HEARTBIT", 0, values)

  @staticmethod
  def mads_status_update(CC: structs.CarControl, CC_SP: structs.CarControlSP, CS) -> MadsDataSP:
    enable_mads = CC_SP.mads.available
    paused = CC_SP.mads.enabled and not CC.latActive

    if any(be.type == ButtonType.lkas and be.pressed for be in CS.out.buttonEvents):
      CS.lkas_disabled = not CS.lkas_disabled

    return MadsDataSP(enable_mads, paused, CS.lkas_disabled)

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS) -> None:
    self.mads = self.mads_status_update(CC, CC_SP, CS)


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.lkas_heartbit = 0

    self.init_lkas_disabled = False
    self.lkas_disabled = False

  @staticmethod
  def get_parser(CP, pt_messages, cam_messages) -> None:
    if CP.carFingerprint in RAM_CARS:
      pt_messages += [
        ("Center_Stack_2", 1),
      ]
    else:
      pt_messages.append(("TRACTION_BUTTON", 1))
      cam_messages.append(("LKAS_HEARTBIT", 1))

  def get_lkas_button(self, cp, cp_cam):
    if self.CP.carFingerprint in RAM_CARS:
      lkas_button = cp.vl["Center_Stack_2"]["LKAS_Button"]
    else:
      lkas_button = cp.vl["TRACTION_BUTTON"]["TOGGLE_LKAS"]
      self.lkas_heartbit = cp_cam.vl["LKAS_HEARTBIT"]
      if not self.init_lkas_disabled:
        self.lkas_disabled = cp_cam.vl["LKAS_HEARTBIT"]["LKAS_DISABLED"]
        self.init_lkas_disabled = True

    return lkas_button

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.prev_lkas_button = self.lkas_button
    self.lkas_button = self.get_lkas_button(cp, cp_cam)
