from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.cam_lka.carcontroller import CarController as CamLkaCarController
from opendbc.car.byd.cam_lka.carstate import CarState as CamLkaCarState
from opendbc.car.byd.mpc_lka.carcontroller import CarController as MpcLkaCarController
from opendbc.car.byd.mpc_lka.carstate import CarState as MpcLkaCarState
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.values import CAR, PLATFORM_MPC_LKA, BydFlags


class CarInterface(CarInterfaceBase):
  RadarInterface = RadarInterface

  def __init__(self, CP):
    if CP.carFingerprint in PLATFORM_MPC_LKA:
      self.CarState = MpcLkaCarState
      self.CarController = MpcLkaCarController
    else:
      self.CarState = CamLkaCarState
      self.CarController = CamLkaCarController
    super().__init__(CP)

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "byd"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]
    # Default 0: no torque spoof/block. Platforms that need spoof set their own param.
    ret.safetyConfigs[0].safetyParam = 0

    ret.steerLimitTimer = 0.6
    ret.steerActuatorDelay = 0.01
    ret.lateralTuning.init("pid")
    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.9871
    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [530]]
    ret.lateralTuning.pid.kpBP = [0.0, 5.0, 20.0]
    ret.lateralTuning.pid.kiBP = [0.0, 5.0, 20.0]
    ret.lateralTuning.pid.kf = 0.00015
    ret.wheelSpeedFactor = 0.66
    # This fork's opendbc base deprecated starting/stopping-state tuning (no other
    # brand here still sets it); EOP10's own cereal/car.capnp still carries these
    # live for selfdrive/controls/lib/longcontrol.py, so keep setting them here via
    # the deprecated group until that cross-schema gap is addressed separately.
    ret.deprecated.startingState = True
    ret.deprecated.startAccel = 3.0
    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.deprecated.stoppingDecelRate = 0.2
    ret.longitudinalActuatorDelay = 0.3

    if candidate in PLATFORM_MPC_LKA:
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
      ret.flags |= int(BydFlags.MPC_LKA | BydFlags.ACC_ON_ESC)
      ret.safetyConfigs[0].safetyParam = 4
      ret.openpilotLongitudinalControl = False
      ret.pcmCruise = True  # stock ACC rising edge triggers pcmEnable → lateral
      ret.radarUnavailable = True
      ret.wheelSpeedFactor = 0.66  # Song Plus wheel speed calibration (1.0 was ~40-50% high vs GPS)
      ret.minSteerSpeed = 0.0
    elif candidate in (CAR.BYD_ATTO3, CAR.BYD_M6, CAR.BYD_SEAL6):
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
      if candidate == CAR.BYD_ATTO3:
        # safetyParam 1: ACC_CMD(814) engage + STEERING_TORQUE spoof (block PT→cam 0x1FC)
        ret.safetyConfigs[0].safetyParam = 1
      elif candidate in (CAR.BYD_M6, CAR.BYD_SEAL6):
        ret.safetyConfigs[0].safetyParam = 3
      if candidate == CAR.BYD_SEAL6:
        ret.pcmCruise = True
        ret.wheelSpeedFactor = 0.6336
        ret.enableBsm = False
    elif candidate in (CAR.BYD_SEAL, CAR.BYD_SEALION7, CAR.BYD_SHARK):
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
      ret.safetyConfigs[0].safetyParam = 2
      ret.openpilotLongitudinalControl = False
      ret.radarUnavailable = True
      ret.wheelSpeedFactor = 0.6336
    else:
      ret.dashcamOnly = True
      ret.safetyModel = structs.CarParams.SafetyModel.noOutput

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    events = self.create_common_events(ret)
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
