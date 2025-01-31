from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase

TransmissionType = car.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = 'psa'
    ret.dashcamOnly = False

    ret.radarUnavailable = True
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2  # not measured
    ret.steerLimitTimer = 1.0

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.manual
      ret.minEnableSpeed = 40 * CV.KPH_TO_MS  # 25mph, FIXME: manual transmission
    ret.minSteerSpeed = 0.  # TODO: verify

    ret.autoResumeSng = ret.minEnableSpeed == -1
    ret.centerToFront = ret.wheelbase * 0.44  # TODO: verify
    ret.wheelSpeedFactor = 1.04
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_adas, self.cp_cam)
    events = self.create_common_events(ret)
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
