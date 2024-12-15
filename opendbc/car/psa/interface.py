from opendbc.car import structs
from opendbc.car.common.conversions  import Conversions as CV
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = 'psa'
    ret.dashcamOnly = False

    ret.radarUnavailable = True
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2  # not measured
    ret.steerLimitTimer = 1.0

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.automatic # TODO: implement manual
      ret.minEnableSpeed = 30 * CV.KPH_TO_MS  # 19 mph, TODO: 40kph/25mph for other variants
    ret.minSteerSpeed = 0.  # TODO: verify

    ret.autoResumeSng = ret.minEnableSpeed == -1
    ret.centerToFront = ret.wheelbase * 0.44  # TODO: verify
    ret.wheelSpeedFactor = 1.04
    return ret

# TODO: check if needed
  # def _update(self, c):
  #   ret = self.CS.update(self.cp, self.cp_adas, self.cp_cam)
  #   events = self.create_common_events(ret)
  #   ret.events = events.to_msg()
  #   return ret

  # def apply(self, c, now_nanos):
  #   return self.CC.update(c, self.CS, now_nanos)
