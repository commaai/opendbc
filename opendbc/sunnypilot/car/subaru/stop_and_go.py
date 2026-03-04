"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import copy
from enum import StrEnum

from opendbc.car import Bus, structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarStateBase
from opendbc.car.subaru.values import SubaruFlags

from opendbc.sunnypilot.car.subaru import subarucan_ext
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP
from opendbc.can.parser import CANParser

_SNG_ACC_MIN_DIST = 3
_SNG_ACC_MAX_DIST = 4.5


class SnGCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.enabled = CP_SP.flags & (SubaruFlagsSP.STOP_AND_GO | SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE)
    self.manual_parking_brake = CP_SP.flags & SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE

    self.last_standstill_frame = 0
    self.epb_resume_frames_remaining = -1
    self.prev_close_distance = 0.0

  def update_epb_resume_sequence(self, should_resume: bool) -> bool:
    if self.manual_parking_brake:
      return False

    if should_resume:
      self.epb_resume_frames_remaining = 15

    send_resume = self.epb_resume_frames_remaining > 0
    if self.epb_resume_frames_remaining > 0:
      self.epb_resume_frames_remaining -= 1

    return send_resume

  def update_stop_and_go(self, CC: structs.CarControl, CS: CarStateBase, frame: int) -> bool:
    """
    Manages stop-and-go functionality for adaptive cruise control (ACC).

    Args:
        CC: Car control data
        CS: Car state data
        frame: Current frame number

    Returns:
        bool: True if resume command should be sent, False otherwise
    """

    if not CC.enabled or not CC.hudControl.leadVisible:
      return False

    close_distance = CS.es_distance_msg["Close_Distance"]
    in_standstill = CS.out.standstill

    if not in_standstill:
      self.last_standstill_frame = frame

    # Check if we've been in standstill long enough
    mpb_standstill_timers = (0.75, 0.8) if self.CP.flags & SubaruFlags.PREGLOBAL else (0.5, 0.55)
    standstill_duration = (frame - self.last_standstill_frame) * DT_CTRL
    in_standstill_hold = standstill_duration > mpb_standstill_timers[0]
    if (frame - self.last_standstill_frame) * DT_CTRL >= mpb_standstill_timers[1]:
      self.last_standstill_frame = frame

    # Car state distance-based conditions (EPB only)
    in_resume_distance = _SNG_ACC_MIN_DIST < close_distance < _SNG_ACC_MAX_DIST
    distance_increasing = close_distance > self.prev_close_distance
    distance_resume_allowed = in_resume_distance and distance_increasing

    if self.manual_parking_brake:
      # Manual parking brake: Direct resume when the standstill hold threshold is reached to prevent ACC fault
      send_resume = in_standstill_hold
    else:
      # EPB: Resume sequence with trigger on distance with lead car increasing
      should_resume = CS.out.standstill and distance_resume_allowed
      send_resume = self.update_epb_resume_sequence(should_resume)

    self.prev_close_distance = close_distance

    return send_resume

  def create_stop_and_go(self, packer, CC: structs.CarControl, CS: CarStateBase, frame: int) -> list[CanData]:
    can_sends = []

    if not self.enabled:
      return can_sends

    send_resume = self.update_stop_and_go(CC, CS, frame)

    can_sends.append(subarucan_ext.create_throttle(packer, self.CP, CS.throttle_msg, send_resume and not self.manual_parking_brake))

    if frame % 2 == 0:
      can_sends.append(subarucan_ext.create_brake_pedal(packer, self.CP, CS.brake_pedal_msg, send_resume and self.manual_parking_brake))

    return can_sends


class SnGCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.brake_pedal_msg: dict[str, float] = {}
    self.throttle_msg: dict[str, float] = {}

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.brake_pedal_msg = copy.copy(cp.vl["Brake_Pedal"])

    if not self.CP.flags & SubaruFlags.HYBRID:
      self.throttle_msg = copy.copy(cp.vl["Throttle"])
