from opendbc.car import structs, uds
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

    return ret

  # TODO: disable radar ECU
  @staticmethod
  def init(CP, can_recv, can_send):
    # ECU name: ARTIV	ARTIV, RADAR_AV_4, LIDAR, ARTIV_UDS	>6B6:696
    tester_present = bytes([0x3E,0x00])
    disable_ecu(can_recv, can_send, bus=1, addr=0x6B6, com_cont_req=tester_present)