"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
from math import exp

from opendbc.car import structs
from opendbc.car.common.basedir import BASEDIR
from opendbc.car.gm.interface import CAR
from opendbc.sunnypilot.car.interfaces import LatControlInputs, NanoFFModel, TorqueFromLateralAccelCallbackTypeTorqueSpace

NEURAL_PARAMS_PATH = os.path.join(BASEDIR, '../sunnypilot/car/torque_data/neural_ff_weights.json')

NON_LINEAR_TORQUE_PARAMS = {
  CAR.CHEVROLET_BOLT_EUV: [2.6531724862969748, 1.0, 0.1919764879840985, 0.009054123646805178],
  CAR.GMC_ACADIA: [4.78003305, 1.0, 0.3122, 0.05591772],
  CAR.CHEVROLET_SILVERADO: [3.29974374, 1.0, 0.25571356, 0.0465122]
}


class CarInterfaceExt:
  def __init__(self, CP: structs.CarParams, CI_Base):
    self.CP = CP
    self.CI_Base = CI_Base
    self.neural_ff_model = None

  def torque_from_lateral_accel_siglin(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                       gravity_adjusted: bool) -> float:
    def sig(val):
      # https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick
      if val >= 0:
        return 1 / (1 + exp(-val)) - 0.5
      else:
        z = exp(val)
        return z / (1 + z) - 0.5

    # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
    # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
    # This has big effect on the stability about 0 (noise when going straight)
    # ToDo: To generalize to other GMs, explore tanh function as the nonlinear
    non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
    assert non_linear_torque_params, "The params are not defined"
    a, b, c, _ = non_linear_torque_params
    steer_torque = (sig(latcontrol_inputs.lateral_acceleration * a) * b) + (latcontrol_inputs.lateral_acceleration * c)
    return float(steer_torque)

  def torque_from_lateral_accel_neural(self, latcontrol_inputs: LatControlInputs, orque_params: structs.CarParams.LateralTorqueTuning,
                                       gravity_adjusted: bool) -> float:
    inputs = list(latcontrol_inputs)
    if gravity_adjusted:
      inputs[0] += inputs[1]
    return float(self.neural_ff_model.predict(inputs))

  def torque_from_lateral_accel_in_torque_space(self) -> TorqueFromLateralAccelCallbackTypeTorqueSpace:
    if self.CP.carFingerprint == CAR.CHEVROLET_BOLT_EUV:
      self.neural_ff_model = NanoFFModel(NEURAL_PARAMS_PATH, self.CP.carFingerprint)
      return self.torque_from_lateral_accel_neural
    elif self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      return self.torque_from_lateral_accel_siglin
    else:
      return self.CI_Base.torque_from_lateral_accel_linear_in_torque_space
