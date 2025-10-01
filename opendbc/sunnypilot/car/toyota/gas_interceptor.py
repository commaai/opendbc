"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.toyota.values import CAR, MIN_ACC_SPEED, PEDAL_TRANSITION
from opendbc.sunnypilot.car import create_gas_interceptor_command


class GasInterceptorCarController:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.gas = 0.
    self.interceptor_gas_cmd = 0.

  def create_gas_command(self, CC: structs.CarControl, CS: structs.CarState, actuators: structs.CarControl.Actuators,
                         packer, frame: int) -> list[CanData]:
    can_sends = []

    if self.CP_SP.enableGasInterceptor and CC.longActive:
      MAX_INTERCEPTOR_GAS = 0.3
      # RAV4 has very sensitive gas pedal
      if self.CP.carFingerprint in (CAR.TOYOTA_RAV4, CAR.TOYOTA_RAV4H, CAR.TOYOTA_HIGHLANDER):
        PEDAL_SCALE = np.interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif self.CP.carFingerprint in (CAR.TOYOTA_COROLLA,):
        PEDAL_SCALE = np.interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = np.interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # offset for creep and windbrake
      pedal_offset = np.interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      self.interceptor_gas_cmd = float(np.clip(pedal_command, 0., MAX_INTERCEPTOR_GAS))
    else:
      self.interceptor_gas_cmd = 0.

    if frame % 2 == 0 and self.CP_SP.enableGasInterceptor and self.CP.openpilotLongitudinalControl:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_interceptor_command(packer, self.interceptor_gas_cmd, frame // 2))
      self.gas = self.interceptor_gas_cmd

    return can_sends
