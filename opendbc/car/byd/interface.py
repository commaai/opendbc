from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.carcontroller import CarController


NetworkLocation = structs.CarParams.NetworkLocation

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "byd"
    # SAFETY_BYD: relay engaged. openpilot owns STEERING_MODULE_ADAS and LKAS_HUD_ADAS on
    # bus 0; panda blocks the camera's copies from being forwarded 2->0 (check_relay) so the
    # EPS and cluster only ever see one source. All other camera messages pass through.
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]
    ret.radarUnavailable = True

    # BYD EPS receives absolute steering wheel angle targets in STEERING_MODULE_ADAS.
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    # Measured on the car (2026-08-07 drive, cross-correlating the transmitted 0x1E2
    # angle against the 0x11F measured angle): the EPS follows with ~0.3-0.8 s of lag.
    # A higher value makes the planner start turning earlier into curves, which reduces
    # curve-entry steerSaturated events on this torque-limited EPS.
    ret.steerActuatorDelay = 0.35
    ret.steerLimitTimer = 0.4

    ret.longitudinalTuning.kpBP = [0., 35.]
    ret.longitudinalTuning.kpV = [1.2, 0.5]
    ret.longitudinalTuning.kiBP = [0., 35.]
    ret.longitudinalTuning.kiV = [0.18, 0.12]

    ret.networkLocation = NetworkLocation.fwdCamera

    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True
    ret.minEnableSpeed = -1
    ret.minSteerSpeed = 0.

    return ret
