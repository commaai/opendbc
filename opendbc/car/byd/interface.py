#!/usr/bin/env python3

from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.values import CAR
from opendbc.car.interfaces import CarInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


class CarInterface(CarInterfaceBase):
    @staticmethod
    def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:  # type: ignore
        ret.carName = "byd"
        ret.safetyConfigs = [get_safety_config(
            structs.CarParams.SafetyModel.byd)]

        ret.dashcamOnly = candidate not in (CAR.BYD_ATTO3)

        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

        ret.experimentalLongitudinalAvailable = False
        ret.radarUnavailable = True

        ret.minEnableSpeed = -1
        ret.minSteerSpeed = 2 * CV.KPH_TO_MS

        # Measured at 0.4s, however in torqued.py, line 55, code will add 0.2
        ret.steerActuatorDelay = 0.2
        ret.steerLimitTimer = 0.4

        ret.lateralTuning.init('pid')
        ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [
            [10., 40.], [10., 40.]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [
            [0.16, 0.33], [0.015, 0.025]]
        ret.lateralTuning.pid.kf = 0.00004

        return ret
