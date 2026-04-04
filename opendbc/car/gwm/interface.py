from opendbc.car import structs, get_safety_config, CanBusBase, Bus
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.carstate import CarState
from opendbc.car.gwm.values import GwmSafetyFlags

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP):
      super().__init__(CP)
      self.lat_active = False
      self.isEPSobeying = True
      self.steer_fault_temporary_counter = 0

  def apply(self, CC, now_nanos):
    self.lat_active = CC.latActive
    return super().apply(CC, now_nanos)

  def update(self, can_packets):
    cp = self.can_parsers[Bus.main]
    self.isEPSobeying = cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] == 1
    self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) if (self.lat_active and not self.isEPSobeying) \
                                          else 0
    ret = super().update(can_packets)
    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100

    return ret

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'gwm'

    cfgs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]

    # If multipanda mapping is detected (offset >= 4), keep the first safety slot
    # as `noOutput` so an internal panda remains silent and the vehicle safety config
    # stays as the last entry (`-1`). This enables external panda to control the vehicle.
    CAN = CanBusBase(None, fingerprint)
    if CAN.offset >= 4:
      cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))

    ret.safetyConfigs = cfgs

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = False

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.LONG_CONTROL.value

      ret.longitudinalActuatorDelay = 0.15
      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stopAccel = -0.55
      ret.stoppingDecelRate = 0.75
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.65]

    return ret
