from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, CAR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    """
    Comparing:
    - FSD 13: 2c912ca5de3b1ee9/000002f7--e9a4e26504
    - FSD 14: 0f79c454f812791a/000000b6--8c7866d3bf
      Differences between FSD 13 and 14:
      New messages in FSD 14:
      - MCU_THCHumanInterfaceRequest (0x032) - length: 8 (DBC (DBCTools tesla_models.dbc): 7)
      - 0x054 - length: 7
      - ID126RearHVStatus (0x126) - length: 5 (DBC shows 7 bytes, may have changed)
      - ID1A5FrontHVStatus (0x1A5) - length: 5 (DBC shows 7 bytes, may have changed)
      - ID20EPARK_sdiFront (0x20E) - length: 3 (DBC shows 8 bytes, may have changed)
      - 0x237 - length: 8
      - MCU_commands (0x248) - length: 3 (DBC (DBCTools tesla_models.dbc): 8)
      - DAS_bodyControls (0x3E9) - length: 8
      - 0x40C - length: 2
      - 0x489 - length: 8
      - 0x7B5 - length: 3
      - 0x7B7 - length: 1
      - 0x7B8 - length: 8
      Missing messages in FSD 14:
      - ID20CVCRIGHT_hvacRequest (0x20C) - length: 2 (DBC shows 8 bytes, may have changed)
      - RCM_status (0x211) - length: 6
      - 0x3A3 - length: 8
      - PCS_alertMatrix (0x3A4) - length: 8
      - DIS_alertMatrix1 (0x3A5) - length: 6 (DBC shows 8 bytes, may have changed)
      - DIS_alertMatrix1 (0x3A6) - length: 6 (DBC (DBCTools tesla_models.dbc): 8; note: other DBCs define DIS_alertMatrix1 at 0x3A5)
      Message length changes:
      - GTW_gatewayStatus (0x031) - FSD 13: 3, FSD 14: 8 (DBC (DBCTools tesla_models.dbc): 8)
      - 0x051 - FSD 13: 3, FSD 14: 8
      - OCS1 (0x34F) - FSD 13: 6, FSD 14: 7 (DBC (DBCTools tesla_models.dbc): 2)
      - DI_locStatus2 (0x4F6) - FSD 13: 7, FSD 14: 8 (DBC (DBCTools tesla_models.dbc): 4)
      Signal differences:
      - EPAS3S_sysStatus: bit 52 is 0 on FSD 13, 1 on FSD 14
      - DAS_bodyControls.DAS_accActive - FSD 13: [0.0], FSD 14: [0.0, 1.0]
      - DAS_bodyControls.DAS_autopilotActive - FSD 13: [0.0], FSD 14: [0.0, 1.0]
      - DAS_bodyControls.DAS_headlightRequest - FSD 13: [0.0], FSD 14: [0.0, 1.0]
      - DAS_bodyControls.DAS_heaterRequest - FSD 13: [0.0], FSD 14: [0.0, 1.0]
      - DAS_bodyControls.DAS_highLowBeamDecision - FSD 13: [0.0], FSD 14: [0.0, 2.0]
      - DAS_bodyControls.DAS_turnIndicatorRequestReason - FSD 13: [0.0], FSD 14: [0.0, 1.0]
      - DAS_bodyControls.DAS_turnIndicatorRequest - FSD 13: [0.0], FSD 14: [0.0, 1.0, 2.0, 3.0]
      - DAS_control.DAS_jerkMax - FSD 13: [0.0, 4.896000000000001], FSD 14: [0.0, 8.636000000000001]
      - DAS_control.DAS_jerkMin - FSD 13: [-5.068, 0.0], FSD 14: [-9.1, 0.0]
      - DAS_status.DAS_autopilotHandsOnState - FSD 13: [0.0, 1.0, 2.0], FSD 14: [0.0, 1.0, 2.0, 3.0]
      - DAS_status2.DAS_lssState - FSD 13: [0.0, 2.0, 3.0], FSD 14: [0.0, 3.0, 5.0, 7.0]
      - DI_state.DI_aebState - FSD 13: [0.0, 2.0, 6.0], FSD 14: [0.0, 2.0, 4.0]
      - DI_state.DI_cruiseState - FSD 13: [0.0, 2.0, 3.0], FSD 14: [0.0, 2.0]
      - DI_state.DI_vehicleHoldState - FSD 13: [0.0, 2.0, 3.0, 6.0, 7.0], FSD 14: [0.0, 2.0, 3.0, 4.0, 7.0]
      - UI_warning.leftBlinkerBlinking - FSD 13: [0.0], FSD 14: [0.0, 1.0, 2.0]
      - UI_warning.rightBlinkerBlinking - FSD 13: [0.0], FSD 14: [0.0, 1.0, 2.0]
      - UI_warning.scrollWheelPressed - FSD 13: [0.0], FSD 14: [0.0, 1.0]
      - VCLEFT_doorStatus.VCLEFT_frontHandlePWM - FSD 13: [0.0, 8.0], FSD 14: [72.0]
      - VCLEFT_doorStatus.VCLEFT_frontLatchStatus - FSD 13: [0.0, 2.0], FSD 14: [2.0]
      - VCLEFT_doorStatus.VCLEFT_frontLatchSwitch - FSD 13: [0.0, 1.0], FSD 14: [1.0]
      - VCLEFT_doorStatus.VCLEFT_frontRelActuatorSwitch - FSD 13: [0.0, 1.0], FSD 14: [1.0]
      - VCLEFT_doorStatus.VCLEFT_mirrorFoldState - FSD 13: [0.0, 2.0], FSD 14: [2.0]
      - VCLEFT_doorStatus.VCLEFT_mirrorHeatState - FSD 13: [0.0, 2.0], FSD 14: [2.0]
      - VCLEFT_doorStatus.VCLEFT_mirrorRecallState - FSD 13: [0.0, 3.0], FSD 14: [3.0]
      - VCLEFT_doorStatus.VCLEFT_rearHandlePWM - FSD 13: [0.0, 4.0], FSD 14: [4.0]
      - VCLEFT_doorStatus.VCLEFT_rearLatchStatus - FSD 13: [0.0, 2.0], FSD 14: [2.0]
      - VCLEFT_doorStatus.VCLEFT_rearLatchSwitch - FSD 13: [0.0, 1.0], FSD 14: [1.0]
      - VCLEFT_doorStatus.VCLEFT_rearRelActuatorSwitch - FSD 13: [0.0, 1.0], FSD 14: [1.0]
      - VCRIGHT_doorStatus.VCRIGHT_rearHandlePWM - FSD 13: [0.0, 34.0], FSD 14: [0.0, 36.0]

    External DBC resources searched:
    - joshwardell/model3dbc (GitHub) - Found: 0x126 (ID126RearHVStatus), 0x1A5 (ID1A5FrontHVStatus), 0x20C (ID20CVCRIGHT_hvacRequest), 0x20E (ID20EPARK_sdiFront)
    - onyx-m2/onyx-m2-dbc (GitHub) - Found: 0x211 (RCM_status), 0x3A4 (PCS_alertMatrix), 0x3A5 (DIS_alertMatrix1)
    - brianman/DBCTools (Azure DevOps) - Found: 0x031 (GTW_gatewayStatus), 0x032 (MCU_THCHumanInterfaceRequest), 0x248 (MCU_commands), 0x34F (OCS1), 0x3A6 (DIS_alertMatrix1), 0x4F6 (DI_locStatus2)
    - CSS Electronics Tesla DBC files and EV data pack
    - tes•LAX CAN Bus Explorer - Tesla CAN bus visualization tool
    - Scan My Tesla community DBC files
    Note: Messages 0x051, 0x054, 0x237, 0x3A3, 0x40C, 0x489, 0x7B5, 0x7B7, 0x7B8 not found in external DBC files searched.
    """

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret
