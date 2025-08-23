"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.enable_radar_tracks import enable_radar_tracks as hyundai_enable_radar_tracks
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import LongitudinalTuningType
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def setup_interfaces(CI: CarInterfaceBase, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                     params_list: list[dict[str, str]], can_recv: CanRecvCallable = None, can_send: CanSendCallable = None) -> None:
  params_dict = {k: v for param in params_list for k, v in param.items()}

  _initialize_custom_longitudinal_tuning(CI, CP, CP_SP, params_dict)
  _initialize_radar_tracks(CP, CP_SP, can_recv, can_send)


def _initialize_custom_longitudinal_tuning(CI: CarInterfaceBase, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
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
