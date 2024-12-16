"""
The MIT License

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Last updated: July 29, 2024
"""

from enum import StrEnum
from collections import namedtuple

from opendbc.car import Bus, structs
from opendbc.car.chrysler.values import RAM_CARS

from opendbc.sunnypilot import SunnypilotParamFlags
from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser

MadsDataSP = namedtuple("MadsDataSP",
                        ["enable_mads", "paused", "lkas_disabled"])

ButtonType = structs.CarState.ButtonEvent.Type


class MadsCarController:
  def __init__(self):
    super().__init__()
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
  def mads_status_update(CC: structs.CarControl, CS) -> MadsDataSP:
    enable_mads = CC.sunnypilotParams & SunnypilotParamFlags.ENABLE_MADS
    paused = CC.madsEnabled and not CC.latActive

    if any(be.type == ButtonType.lkas and be.pressed for be in CS.out.buttonEvents):
      CS.lkas_disabled = not CS.lkas_disabled

    return MadsDataSP(enable_mads, paused, CS.lkas_disabled)

  def update(self, CC: structs.CarControl, CS) -> None:
    self.mads = self.mads_status_update(CC, CS)


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams):
    super().__init__(CP)
    self.lkas_heartbit = 0

    self.init_lkas_disabled = False
    self.lkas_disabled = False

  @staticmethod
  def get_parser(CP, pt_messages, cam_messages) -> None:
    if CP.carFingerprint in RAM_CARS:
      pt_messages += [
        ("Center_Stack_1", 1),
        ("Center_Stack_2", 1),
      ]
    else:
      pt_messages.append(("TRACTION_BUTTON", 1))
      cam_messages.append(("LKAS_HEARTBIT", 1))

  def get_lkas_button(self, cp, cp_cam):
    if self.CP.carFingerprint in RAM_CARS:
      lkas_button = cp.vl["Center_Stack_1"]["LKAS_Button"] or cp.vl["Center_Stack_2"]["LKAS_Button"]
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
