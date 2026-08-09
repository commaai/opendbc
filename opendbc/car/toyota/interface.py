from opendbc.car import structs, get_safety_config, uds
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import ToyotaFlags, CarControllerParams, MIN_ACC_SPEED, ToyotaSafetyFlags
from opendbc.car.toyota.support_groups import ToyotaAccGroup, ToyotaLateralGroup, ToyotaLongitudinalGroup, ToyotaPtGroup, ToyotaSupportGroups, \
                                                       apply_support_groups
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.interfaces import CarInterfaceBase

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  DRIVABLE_GEARS = (structs.CarState.GearShifter.sport,)

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "toyota"
    groups = ToyotaSupportGroups.from_platform(candidate, fingerprint, car_fw)
    apply_support_groups(ret, groups)
    openpilot_longitudinal = groups.tss2 and (groups.acc != ToyotaAccGroup.RADAR or alpha_long)

    # Protocol-compatible Toyotas share one deliberately generic dynamics
    # prior. paramsd/torqued learn the effective steering model online.
    ret.mass = 1700.
    ret.wheelbase = 2.75
    ret.steerRatio = 15.0
    ret.tireStiffnessFactor = 0.55
    ret.maxLateralAccel = 2.0

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = groups.eps_torque_scale

    # BRAKE_MODULE is on a different address for these cars
    if groups.pt == ToyotaPtGroup.NEW_MC:
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.ALT_BRAKE.value

    if groups.secoc:
      ret.secOcRequired = True
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.SECOC.value
      ret.dashcamOnly = is_release

    if groups.lateral == ToyotaLateralGroup.LTA_ANGLE:
      ret.steerControlType = SteerControlType.angle
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.LTA.value

      # LTA control can be more delayed and winds up more often
      ret.steerActuatorDelay = 0.18
      ret.steerLimitTimer = 0.8
    else:
      # Generic initialization only; liveTorqueParameters learns the vehicle.
      CarInterfaceBase.configure_torque_tune("TOYOTA_RAV4_TSS2", ret.lateralTuning)

      ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
      ret.steerLimitTimer = 0.4

    if groups.hybrid:
      ret.flags |= ToyotaFlags.HYBRID.value

    ret.wheelSpeedFactor = groups.wheel_speed_factor

    # EPS firmware attests exceptional rack behavior independently of identity.
    for fw in car_fw:
      if fw.ecu == "eps" and fw.fwVersion.startswith(b'8965B470') and fw.fwVersion != b'8965B47060\x00\x00\x00\x00\x00\x00':
        CarInterfaceBase.configure_torque_tune("TOYOTA_RAV4_TSS2", ret.lateralTuning, steering_angle_deadzone_deg=0.2)
      if fw.ecu == "eps" and fw.fwVersion == b'8965B47070\x00\x00\x00\x00\x00\x00':
        ret.dashcamOnly = True

    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: Some TSS-P platforms have BSM, but are flipped based on region or driving direction.
    # Detect flipped signals and enable for C-HR and others
    ret.enableBsm = 0x3F6 in fingerprint[0] and groups.tss2

    ret.radarUnavailable = groups.acc == ToyotaAccGroup.RADAR

    # since we don't yet parse radar on TSS2 radar-based ACC cars, gate longitudinal behind alpha toggle
    if groups.longitudinal == ToyotaLongitudinalGroup.RADAR_ACC or bool(ret.flags & ToyotaFlags.RADAR_ACC):
      ret.alphaLongitudinalAvailable = True

      if alpha_long:
        ret.flags |= ToyotaFlags.DISABLE_RADAR.value

    # openpilot longitudinal enabled by default:
    #  - TSS2 cars with camera sending ACC_CONTROL where we can block it
    # openpilot longitudinal behind alpha long toggle:
    #  - TSS2 radar ACC cars (disables radar)

    ret.openpilotLongitudinalControl = openpilot_longitudinal

    ret.autoResumeSng = ret.openpilotLongitudinalControl

    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if groups.stop_and_go else MIN_ACC_SPEED

    if groups.raised_accel_limit:
      ret.flags |= ToyotaFlags.RAISED_ACCEL_LIMIT.value

      # Hybrids have much quicker longitudinal actuator response
      if ret.flags & ToyotaFlags.HYBRID.value:
        ret.longitudinalActuatorDelay = 0.05

    return ret

  @staticmethod
  def init(CP, can_recv, can_send, communication_control=None):
    # disable radar if alpha longitudinal toggled on radar-ACC car
    if CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if communication_control is None:
        communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
      disable_ecu(can_recv, can_send, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    # re-enable radar if alpha longitudinal toggled on radar-ACC car
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX, uds.MESSAGE_TYPE.NORMAL])
    CarInterface.init(CP, can_recv, can_send, communication_control)
