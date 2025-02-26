from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.values import CAR
from opendbc.car.disable_ecu import disable_ecu

TransmissionType = structs.CarParams.TransmissionType

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, experimental_long, docs):
    ret.brand = 'psa'
    ret.dashcamOnly = False

    ret.radarUnavailable = True
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 1.0

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.automatic
      ret.minEnableSpeed = 0
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1
    ret.centerToFront = ret.wheelbase * 0.44
    ret.wheelSpeedFactor = 1.04

    return ret

  # TODO: find radar ECU address to disable it
  # @staticmethod
  # def init(CP, CP_SP, can_recv, can_send):

    # communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
    # disable_ecu(can_recv, can_send, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)