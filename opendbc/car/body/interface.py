import os
import math
from opendbc.car import get_safety_config, structs
from opendbc.car.body.carcontroller import CarController
from opendbc.car.body.carstate import CarState
from opendbc.car.body.values import SPEED_FROM_RPM
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.body.flash import update

FIRMWARE_VERSION = "v0.3.1"
BIN_URL = f"https://github.com/commaai/body/releases/download/{FIRMWARE_VERSION}/body.bin.signed"
BIN_NAME = f"body-v1-{FIRMWARE_VERSION}.bin.signed"
BODY_DIR = os.path.dirname(os.path.realpath(__file__))
BIN_PATH = os.path.join(BODY_DIR, BIN_NAME)

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def init(CP, can_recv, can_send, communication_control=None):
    update(0x250, 0, BIN_PATH, BIN_URL, can_send, can_recv, 0x720, 0x728, 0)

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
