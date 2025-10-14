"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import numpy as np
from typing import NamedTuple
from collections.abc import Callable

from opendbc.car import structs
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.subaru.values import SubaruFlags
from opendbc.sunnypilot.car.hyundai.enable_radar_tracks import enable_radar_tracks as hyundai_enable_radar_tracks
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import LongitudinalTuningType
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP, SubaruSafetyFlagsSP


class LatControlInputs(NamedTuple):
  lateral_acceleration: float
  roll_compensation: float
  vego: float
  aego: float


TorqueFromLateralAccelCallbackTypeTorqueSpace = Callable[[LatControlInputs, structs.CarParams.LateralTorqueTuning, bool], float]


class CarInterfaceBaseSP:
  @staticmethod
  def torque_from_lateral_accel_linear_in_torque_space(latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                                        gravity_adjusted: bool) -> float:
    # The default is a linear relationship between torque and lateral acceleration (accounting for road roll and steering friction)
    return latcontrol_inputs.lateral_acceleration / float(torque_params.latAccelFactor)

  def torque_from_lateral_accel_in_torque_space(self) -> TorqueFromLateralAccelCallbackTypeTorqueSpace:
    return self.torque_from_lateral_accel_linear_in_torque_space


class NanoFFModel:
  def __init__(self, weights_loc: str, platform: str):
    self.weights_loc = weights_loc
    self.platform = platform
    self.load_weights(platform)

  def load_weights(self, platform: str):
    with open(self.weights_loc) as fob:
      self.weights = {k: np.array(v) for k, v in json.load(fob)[platform].items()}

  def relu(self, x: np.ndarray):
    return np.maximum(0.0, x)

  def forward(self, x: np.ndarray):
    assert x.ndim == 1
    x = (x - self.weights['input_norm_mat'][:, 0]) / (self.weights['input_norm_mat'][:, 1] - self.weights['input_norm_mat'][:, 0])
    x = self.relu(np.dot(x, self.weights['w_1']) + self.weights['b_1'])
    x = self.relu(np.dot(x, self.weights['w_2']) + self.weights['b_2'])
    x = self.relu(np.dot(x, self.weights['w_3']) + self.weights['b_3'])
    x = np.dot(x, self.weights['w_4']) + self.weights['b_4']
    return x

  def predict(self, x: list[float], do_sample: bool = False):
    x = self.forward(np.array(x))
    if do_sample:
      pred = np.random.laplace(x[0], np.exp(x[1]) / self.weights['temperature'])
    else:
      pred = x[0]
    pred = pred * (self.weights['output_norm_mat'][1] - self.weights['output_norm_mat'][0]) + self.weights['output_norm_mat'][0]
    return pred


def setup_interfaces(CI, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                     params_list: list[dict[str, str]], can_recv: CanRecvCallable = None, can_send: CanSendCallable = None) -> None:
  params_dict = {k: v for param in params_list for k, v in param.items()}

  _initialize_custom_longitudinal_tuning(CI, CP, CP_SP, params_dict)
  _initialize_radar_tracks(CP, CP_SP, can_recv, can_send)
  _initialize_stop_and_go(CP, CP_SP, params_dict)


def _initialize_custom_longitudinal_tuning(CI, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                           params_dict: dict[str, str]) -> None:

  # Hyundai Custom Longitudinal Tuning
  if CP.brand == 'hyundai':
    hyundai_longitudinal_tuning = int(params_dict["HyundaiLongitudinalTuning"])
    if hyundai_longitudinal_tuning == LongitudinalTuningType.DYNAMIC:
      CP_SP.flags |= HyundaiFlagsSP.LONG_TUNING_DYNAMIC.value
    if hyundai_longitudinal_tuning == LongitudinalTuningType.PREDICTIVE:
      CP_SP.flags |= HyundaiFlagsSP.LONG_TUNING_PREDICTIVE.value

  _ = CI.get_longitudinal_tuning_sp(CP, CP_SP)


def _initialize_radar_tracks(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv: CanRecvCallable = None, can_send: CanSendCallable = None) -> None:
  if CP.brand == 'hyundai':
    if CP.flags & HyundaiFlags.MANDO_RADAR and (CP.radarUnavailable or CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC):
      tracks_enabled = hyundai_enable_radar_tracks(can_recv, can_send, bus=0, addr=0x7d0)
      CP.radarUnavailable = not tracks_enabled


def _initialize_stop_and_go(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  if CP.brand == 'subaru' and not CP.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID):
    stop_and_go = int(params_dict["SubaruStopAndGo"]) == 1
    stop_and_go_manual_parking_brake = int(params_dict["SubaruStopAndGoManualParkingBrake"]) == 1

    if stop_and_go:
      CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO.value
    if stop_and_go_manual_parking_brake:
      CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE.value
    if stop_and_go or stop_and_go_manual_parking_brake:
      CP_SP.safetyParam |= SubaruSafetyFlagsSP.STOP_AND_GO
