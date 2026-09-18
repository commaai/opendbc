from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, DBC
from opendbc.car.tesla.radar_interface import RadarInterface, RADAR_START_ADDR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    if 0x209 not in fingerprint[CANBUS.autopilot_party] and 0x7ff not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.HW_2_5.value

    # Model X and HW 2.5 vehicles are missing DAS_settings
    if 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # Tesla expanded DAS_steeringControl->DAS_steeringControlType to 3 bits: first in the FSD 14 builds for HW4 around 10-26-2025,
    # then in the other HW4 builds around 03-02-2026, and for HW3 and HW2.5 with 2026.8.6 around 04-03-2026.
    # - 0x489 is only sent by HW3/HW4 Autopilot computers on the 3-bit firmware
    # - 0x054 is sent by the car on the 3-bit firmware, so it also detects HW2.5 vehicles
    if 0x489 in fingerprint[CANBUS.autopilot_party] or 0x054 in fingerprint[CANBUS.party]:
      ret.flags |= TeslaFlags.DAS_STEERING_3_BIT.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.DAS_STEERING_3_BIT.value

    # Radar support is intended to work for:
    # - Tesla Model 3 vehicles built approximately mid-2017 through early-2021
    # - Tesla Model Y vehicles built approximately mid-2020 through early-2021
    # - Vehicles equipped with the Continental ARS4-B radar (used on HW2 / HW2.5 / early HW3)
    # - Radar CAN lines must be tapped and connected to CAN bus 1 (normally not used for tesla vehicles)
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in DBC[candidate]

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X,)  # dashcam only, pending find invalidLkasSetting signal

    return ret
