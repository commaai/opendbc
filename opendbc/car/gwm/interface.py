from opendbc.car import structs, get_safety_config, Bus, create_button_events
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.carstate import CarState
from opendbc.car.gwm.values import GwmSafetyFlags

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP):
      super().__init__(CP)
      self.lat_active = False
      self.isEPSobeying = True
      self.steer_fault_temporary_counter = 0
      self.current_personality = 0
      self.pcm_follow_distance = 0
      self.press_gac_button = False

  def apply(self, CC, now_nanos):
    self.lat_active = CC.latActive
    hud_control = CC.hudControl
    self.current_personality = hud_control.leadDistanceBars
    return super().apply(CC, now_nanos)

  def update(self, can_packets):
    cp = self.can_parsers[Bus.main]
    self.isEPSobeying = cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] == 1
    self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) if (self.lat_active and not self.isEPSobeying) \
                                          else 0

    cp_cam = self.can_parsers[Bus.cam]
    self.pcm_follow_distance = cp_cam.vl["ACC"]["CAR_DISTANCE_SELECTION"]

    ret = super().update(can_packets)
    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100
    if (self.pcm_follow_distance == 4 and self.current_personality != 3) or \
       (self.pcm_follow_distance == 3 and self.current_personality != 3) or \
       (self.pcm_follow_distance == 2 and self.current_personality != 2) or \
       (self.pcm_follow_distance == 1 and self.current_personality != 1):
      self.press_gac_button = not self.press_gac_button
    ret.buttonEvents = create_button_events(self.press_gac_button, True, {1: ButtonType.gapAdjustCruise})

    return ret

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'gwm'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = False

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.LONG_CONTROL.value

      ret.longitudinalActuatorDelay = 0.25
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25
      ret.stopAccel = -0.75
      ret.stoppingDecelRate = 0.75
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.4]

    return ret
