from opendbc.car import Bus, structs, get_safety_config, uds
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, MIN_ACC_SPEED, \
                                                  EPS_SCALE, ToyotaSafetyFlags
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
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = EPS_SCALE[candidate]

    # BRAKE_MODULE is on a different address for these cars
    if DBC[candidate][Bus.pt] == "toyota_new_mc_pt_generated":
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.ALT_BRAKE.value

    if ret.flags & ToyotaFlags.SECOC.value:
      ret.secOcRequired = True
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.SECOC.value
      ret.dashcamOnly = is_release

    if ret.flags & ToyotaFlags.ANGLE_CONTROL:
      ret.steerControlType = SteerControlType.angle
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.LTA.value

      # LTA control can be more delayed and winds up more often
      ret.steerActuatorDelay = 0.18
      ret.steerLimitTimer = 0.8
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
      ret.steerLimitTimer = 0.4

    stop_and_go = bool(ret.flags & ToyotaFlags.TSS2)

    # In TSS2 cars, the camera does long control
    found_ecus = [fw.ecu for fw in car_fw]

    if Ecu.hybrid in found_ecus:
      ret.flags |= ToyotaFlags.HYBRID.value

    if candidate == CAR.TOYOTA_PRIUS:
      stop_and_go = True
      # Only give steer angle deadzone to for bad angle sensor prius
      for fw in car_fw:
        if fw.ecu == "eps" and not fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00':
          ret.steerActuatorDelay = 0.25
          CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)
        # 2021+ TSS2 steering rack swapped into a TSS-P car, not supported
        if fw.ecu == "eps" and fw.fwVersion == b'8965B47070\x00\x00\x00\x00\x00\x00':
          ret.dashcamOnly = True

    elif candidate in (CAR.LEXUS_RX, CAR.LEXUS_RX_TSS2):
      stop_and_go = True
      ret.wheelSpeedFactor = 1.035

    elif candidate in (CAR.TOYOTA_AVALON, CAR.TOYOTA_AVALON_2019, CAR.TOYOTA_AVALON_TSS2):
      # starting from 2019, all Avalon variants have stop and go
      # https://engage.toyota.com/static/images/toyota_safety_sense/TSS_Applicability_Chart.pdf
      stop_and_go = candidate != CAR.TOYOTA_AVALON

    elif candidate in (CAR.TOYOTA_CHR, CAR.TOYOTA_CAMRY, CAR.TOYOTA_SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_LS, CAR.LEXUS_NX):
      # TODO: Some of these platforms are not advertised to have full range ACC, do they really all have sng?
      stop_and_go = True

    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: Some TSS-P platforms have BSM, but are flipped based on region or driving direction.
    # Detect flipped signals and enable for C-HR and others
    if 0x3F6 in fingerprint[0] and ret.flags & ToyotaFlags.TSS2:
      ret.flags |= ToyotaFlags.HAS_BSM.value

    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    # since we don't yet parse radar on TSS2 radar-based ACC cars, gate longitudinal behind alpha toggle
    if ret.flags & ToyotaFlags.RADAR_ACC:
      ret.alphaLongitudinalAvailable = True

      if alpha_long:
        ret.flags |= ToyotaFlags.DISABLE_RADAR.value

    # openpilot longitudinal enabled by default:
    #  - TSS2 cars with camera sending ACC_CONTROL where we can block it
    # openpilot longitudinal behind alpha long toggle:
    #  - TSS2 radar ACC cars (disables radar)

    ret.openpilotLongitudinalControl = ((bool(ret.flags & ToyotaFlags.TSS2) and not (ret.flags & ToyotaFlags.RADAR_ACC)) or
                                        bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value))

    ret.autoResumeSng = ret.openpilotLongitudinalControl

    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if stop_and_go else MIN_ACC_SPEED

    if ret.flags & ToyotaFlags.TSS2:
      ret.flags |= ToyotaFlags.RAISED_ACCEL_LIMIT.value

      # Hybrids have much quicker longitudinal actuator response
      if ret.flags & ToyotaFlags.HYBRID.value:
        ret.longitudinalActuatorDelay = 0.05

    # TSS 3.0 (CAN FD) -- Phase 1, read only. This must come LAST so it overrides
    # every default set above, in particular openpilotLongitudinalControl, which
    # the TSS2 rule turns on for this platform.
    #
    # noOutput is "like silent but without silent CAN TXs": the panda receives
    # everything and transmits nothing. Combined with dashcamOnly staying False,
    # this is exactly the port doc's section 7 item 5 -- the driving model runs
    # and renders lanes/path, and no actuation is physically possible.
    if ret.flags & ToyotaFlags.CAN_FD.value:
      # Lateral is impossible on this car regardless of any toggle: the
      # camera->EPS command rides on the untapped CA2 bus and does not exist in
      # this DBC. Longitudinal (0x13C) IS plaintext on an already-tapped bus, so
      # it is offered as opt-in behind the standard Alpha Longitudinal toggle --
      # a param, so sunnylink can set it like any other.
      # The toggle is only offered once the path is at least in shadow mode --
      # a toggle that does nothing is worse than no toggle.
      # NO STOP-AND-GO. The platform inherits stop_and_go from the TSS2 flag,
      # which would set minEnableSpeed = -1 and let openpilot drive the car to a
      # standstill -- but the ACC standstill/hold state is not decoded yet
      # (cruiseState.standstill is hardcoded False, port doc 12.2 item 1).
      # Capping the low end is what makes longitudinal usable WITHOUT it:
      # car_events.py raises belowEngageSpeed under minEnableSpeed and
      # speedTooLow (which disengages) if the planner still wants throttle, so
      # openpilot hands back before standstill is ever reached and
      # cruiseState.standstill is never consulted.
      # 19 mph == Toyota MIN_ACC_SPEED, and matches this car's own set-speed floor.
      ret.minEnableSpeed = MIN_ACC_SPEED

      # TSS3 longitudinal rides on the camera-origin 0x160 modify-and-forward path, so
      # openpilot longitudinal is always available here (not gated behind the alpha-
      # longitudinal param, which is auto-deleted offroad).
      ret.alphaLongitudinalAvailable = False
      ret.openpilotLongitudinalControl = True
      ret.autoResumeSng = False
      # Lateral: angle control via the LTA path (steer request in 0x160 bytes 22-23).
      ret.steerControlType = SteerControlType.angle

      # Toyota safety with the TSS3 flag, which permits the 0x160 modify-and-forward tx
      # (see opendbc/safety/modes/toyota.h). The carcontroller only substitutes into
      # 0x160 while the stock ACC is engaged (cruiseState.enabled == the panda's
      # controls_allowed source); otherwise it relays the camera's own frame. Requires
      # a panda built with ALLOW_DEBUG (the TSS3 param is debug-gated, like SECOC).
      ret.safetyConfigs[0].safetyParam &= ~ToyotaSafetyFlags.STOCK_LONGITUDINAL.value
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.TSS3.value

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
