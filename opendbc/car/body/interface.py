import math
from opendbc.car import get_safety_config, structs
from opendbc.car.body.carcontroller import CarController
from opendbc.car.body.carstate import CarState
from opendbc.car.body.values import SPEED_FROM_RPM, FIRMWARE_VERSION, BIN_PATH, BIN_URL, FLASH_ADDR, UDS_TX, UDS_RX, BUS
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.body.flash import update
from opendbc.car.fw_query_definitions import StdQueries


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def init(CP, can_recv, can_send, communication_control=None):
    fw_signature = next(
      fw.fwVersion for fw in CP.carFw
      if fw.ecu == structs.CarParams.Ecu.engine
      and fw.request[1] == StdQueries.APPLICATION_SOFTWARE_FINGERPRINT_REQUEST
    )
    update(can_send, can_recv, FLASH_ADDR, BUS, BIN_PATH, BIN_URL, fw_signature)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.notCar = True
    ret.brand = "body"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.body)]

    ret.minSteerSpeed = -math.inf
    ret.maxLateralAccel = math.inf  # TODO: set to a reasonable value
    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.

    ret.wheelSpeedFactor = SPEED_FROM_RPM

    ret.radarUnavailable = True
    ret.openpilotLongitudinalControl = True
    ret.steerControlType = structs.CarParams.SteerControlType.angle

    return ret
