from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, DBC, FSD_14_FW, Ecu
from opendbc.car.tesla.radar_interface import BROWNPANDA_RADAR_CARS, RadarInterface, RADAR_START_ADDR


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

    # Model X and HW 2.5 vehicles are missing DAS_settings
    if 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # Radar support is intended to work for:
    # - Tesla Model 3 vehicles built approximately mid-2017 through early-2021
    # - Tesla Model Y vehicles built approximately mid-2020 through early-2021
    # - Vehicles equipped with the Continental ARS4-B radar (used on HW2 / HW2.5 / early HW3)
    # - Radar CAN lines must be tapped and connected to CAN bus 1 (normally not used for tesla vehicles)
    #
    # BrownPanda-gatewayed cars have no bus-1 tap and instead advertise the
    # same Continental ARS4-B layout on logical party bus 0 (see
    # opendbc/car/tesla/radar_interface.py + docs/BROWNPANDA_RADAR.md).
    # Either source being present is sufficient.
    native_radar_present = RADAR_START_ADDR in fingerprint[1] and Bus.radar in DBC[candidate]
    party_fingerprint = fingerprint.get(CANBUS.party, {})
    brownpanda_radar_present = party_fingerprint.get(0x401) == 8 and party_fingerprint.get(0x45F) == 8
    brownpanda_radar_available = candidate in BROWNPANDA_RADAR_CARS and brownpanda_radar_present
    ret.radarUnavailable = not (native_radar_present or brownpanda_radar_available)

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    fsd_14 = any(fw.ecu == Ecu.eps and fw.fwVersion in FSD_14_FW.get(candidate, []) for fw in car_fw)
    if fsd_14:
      ret.flags |= TeslaFlags.FSD_14.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.FSD_14.value

    ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X,)  # dashcam only, pending find invalidLkasSetting signal

    return ret
