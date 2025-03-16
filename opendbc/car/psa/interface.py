from opendbc.car import structs, uds
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.values import CAR
from opendbc.car.disable_ecu import disable_ecu

TransmissionType = structs.CarParams.TransmissionType

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = 'psa'
    ret.dashcamOnly = False

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 1.0

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.automatic
      ret.minEnableSpeed = 0
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1
    ret.centerToFront = ret.wheelbase * 0.44
    ret.wheelSpeedFactor = 1.04

    # longitudinal
    ret.radarUnavailable = True
    ret.experimentalLongitudinalAvailable = True

    # TODO: check where experimental_long is set
    # if experimental_long:
    ret.openpilotLongitudinalControl = True
    ret.vEgoStopping = 0.1
    ret.vEgoStarting = 0.1
    ret.stoppingDecelRate = 0.3

    return ret

  # TODO: disable radar ECU
  @staticmethod
  def init(CP, can_recv, can_send):
    # ECU name: ARTIV	ARTIV, RADAR_AV_4, LIDAR, ARTIV_UDS	>6B6:696
    communication_control = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.PROGRAMMING])
    disable_ecu(can_recv, can_send, bus=1, addr=0x6b6, com_cont_req=communication_control)