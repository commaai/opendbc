from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.disable_ecu import disable_ecu

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = 'psa'
    ret.dashcamOnly = False

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.automatic
      ret.minEnableSpeed = 0
    ret.minSteerSpeed = 0.

    ret.centerToFront = ret.wheelbase * 0.44
    ret.wheelSpeedFactor = 1.04

    # longitudinal
    ret.radarUnavailable = True

    # TODO: check where experimental_long is set
    # if experimental_long:
    # TODO DELETE
    print(f"*************experimental long: {experimental_long}******************")
    # ret.longitudinalTuning.kiBP = [0., 35.]
    # ret.longitudinalTuning.kiV = [1.5, 0.8]

    # TODO DELETE

    ret.openpilotLongitudinalControl = False
    ret.experimentalLongitudinalAvailable = True
    # ret.longitudinalActuatorDelay = 0.5
    # TODO: tune
    # ret.stopAccel = -10.65
    # ret.startAccel = 1.0
    # ret.vEgoStopping = 0.1
    # ret.vEgoStarting = 0.1
    # ret.stoppingDecelRate = 2.0
    return ret

  # # TODO: disable radar ECU here instead of carcontroller
  # @staticmethod
  # def init(CP, can_recv, can_send):
  #   # ECU name: ARTIV	ARTIV, RADAR_AV_4, LIDAR, ARTIV_UDS	>6B6:696
  #   communication_control = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.PROGRAMMING])
  #   disable_ecu(can_recv, can_send, bus=1, addr=0x6b6, com_cont_req=communication_control)