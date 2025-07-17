import math
import numpy as np
from opendbc.car import Bus, apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                        make_tester_present_msg, rate_limit, structs, ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.secoc import add_mac, build_sync_mac
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker

Ecu = structs.CarParams.Ecu
LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# The up limit allows the brakes/gas to unwind quickly leaving a stop,
# the down limit roughly matches the rate of ACCEL_NET, reducing PCM compensation windup
ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_PID_UNWIND = 0.03 * DT_CTRL * 3  # m/s^2 / frame

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500


class LeadLagFilter:
  """
  Drivability lead‑lag network:
          (1 + τ s)
  G(s) = ------------
          (1 + β τ s)

  • τ   ~= PCM time‑constant (0.3‑0.6 s on Camry)
  • β   < 1   (0.05‑0.2)  sets how much high‑freq you keep
  """
  def __init__(self, tau: float, beta: float, dt: float):
    wz = 1.0 / tau                 # rad/s of the zero
    wp = wz * beta                 # slower pole

    # bilinear‑transform coefficients
    self._a = (2 + wp*dt) / (2 + wz*dt)
    self._b = (2 - wp*dt) / (2 + wp*dt)
    self._c = (2 - wz*dt) / (2 + wz*dt)

    self._u1 = 0.0                 # previous input  (u[k-1])
    self._y1 = 0.0                 # previous output (y[k-1])
    self._k = (1 + self._b) / (self._a + self._c)

  def __call__(self, u: float) -> float:
    y = self._a * u + self._c * self._u1 - self._b * self._y1
    self._u1, self._y1 = u, y
    return self._k * y


# class HP_LP_Drivability:
#   """
#   High‑pass (tau_hp) immediately followed by
#   Low‑pass  (tau_lp)  → behaves like a lead‑lag
#   """
#   def __init__(self, tau_hp, tau_lp, dt):
#     self.lp_tail = FirstOrderFilter(0, tau_lp, dt)
#     self.hp_state = FirstOrderFilter(0, tau_hp, dt)  # used only for its internal y
#     self.y_prev = 0.0
#
#   def __call__(self, u):
#     # --- high‑pass via y_hp = u - lowpass(u) ---
#     lp_u = self.hp_state.update(u)
#     y_hp = u - lp_u                # quick “kick” part
#
#     # --- add kick to previous output, then low‑pass the sum ---
#     blended = self.y_prev + y_hp   # this is the lead action
#     y_out = self.lp_tail.update(blended)  # lag smooth back
#     self.y_prev = y_out
#     return y_out

class LeadLagSimple:
  """
  Discrete approximation of  (1 + τ s) / (1 + β τ s)
  implemented as:  y = u + (1‑β)·HPβτ(u)
  where HPβτ is a first‑order high‑pass with time–constant β·τ
  """
  def __init__(self, tau: float, beta: float, dt: float):
    self.beta = beta
    self.tau_hp = beta * tau               # HP time‑constant
    self.alpha  = dt / (self.tau_hp + dt)  # LP coefficient inside HP
    self.lp     = 0.0                      # internal low‑pass state

  def __call__(self, u: float) -> float:
    # first‑order low‑pass (part of the high‑pass)
    self.lp += self.alpha * (u - self.lp)
    hp = u - self.lp                       # high‑pass output

    # lead‑lag output: input + scaled HP
    return u + (1.0 - self.beta) * hp


def get_long_tune(CP, params):
  if CP.carFingerprint in TSS2_CAR:
    kiBP = [2., 5.]
    kiV = [0.5, 0.25]
  else:
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]

  return PIDController(0.0, (kiBP, kiV), k_f=1.0,
                       pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
                       rate=1 / (DT_CTRL * 3))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.last_torque = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.distance_button = 0

    self.prev_accel_scurve = 0

    tau_pcm  = 0.25          # <-- tune from step log
    beta     = 0.05          # <-- tune (start 0.10)
    self.drv_filter = LeadLagFilter(tau_pcm, beta, DT_CTRL*3)

    # # simple version of the drivability filter
    # dt = DT_CTRL * 3
    # self.drv_filter_simple = HP_LP_Drivability(tau_hp=0.15, tau_lp=0.45, dt=dt)

    # tau_pcm = 0.45  # same τ you were using
    # beta = 0.12  # same β
    # self.drv_filter_simple = LeadLagSimple(tau_pcm, beta, DT_CTRL * 3)

    # *** start long control state ***
    self.long_pid = get_long_tune(self.CP, self.params)
    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 3)
    self.pitch = FirstOrderFilter(0, 0.25, DT_CTRL)
    self.pitch_slow = FirstOrderFilter(0, 1.5, DT_CTRL)

    self.accel_filter = FirstOrderFilter(0.0, 0.4, DT_CTRL * 3)
    self.accel_filter_slow = FirstOrderFilter(0.0, 1, DT_CTRL * 3)

    self.accel_filter2 = FirstOrderFilter(0.0, 0.05, DT_CTRL * 3)  # TODO: dont use this filter?
    self.accel_filter_slow2 = FirstOrderFilter(0.0, 0.2, DT_CTRL * 3)

    self.debug = 0.0
    self.debug2 = 0.0
    self.debug3 = 0.0
    self.debug4 = 0.0
    self.debug5 = 0.0

    self.accel = 0
    self.prev_accel = 0
    # *** end long control state ***

    self.packer = CANPacker(dbc_names[Bus.pt])

    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_prev_reset_counter = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])
      self.pitch_slow.update(CC.orientationNED[1])

    # *** control msgs ***
    can_sends = []

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        expected_mac = build_sync_mac(self.secoc_key, int(CS.secoc_synchronization['TRIP_CNT']), int(CS.secoc_synchronization['RESET_CNT']))
        if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac:
          carlog.error("SecOC synchronization MAC mismatch, wrong key?")

    # *** steer torque ***
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_meas_steer_torque_limits(new_torque, self.last_torque, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_torque = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_torque = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        self.last_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw,
                                                       CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg,
                                                       CC.latActive, self.params.ANGLE_LIMITS)

    self.last_torque = apply_torque

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    steer_command = toyotacan.create_steer_command(self.packer, apply_torque, apply_steer_req)
    if self.CP.flags & ToyotaFlags.SECOC.value:
      # TODO: check if this slow and needs to be done by the CANPacker
      steer_command = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lka_message_counter,
                              steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < self.params.MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

      if self.CP.flags & ToyotaFlags.SECOC.value:
        lta_steer_2 = toyotacan.create_lta_steer_command_2(self.packer, self.frame // 2)
        lta_steer_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lta_message_counter,
                              lta_steer_2)
        self.secoc_lta_message_counter += 1
        can_sends.append(lta_steer_2)

    # *** gas and brake ***

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR):
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_standstill = CS.out.standstill

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 3 == 0:
        # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
        if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
          desired_distance = 4 - hud_control.leadDistanceBars
          if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
            self.distance_button = not self.distance_button
          else:
            self.distance_button = 0

        # internal PCM gas command can get stuck unwinding from negative accel so we apply a generous rate limit
        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
        self.prev_accel = pcm_accel_cmd

        # calculate amount of acceleration PCM should apply to reach target, given pitch.
        # clipped to only include downhill angles, avoids erroneously unsetting PERMIT_BRAKING when stopping on uphills
        accel_due_to_pitch = math.sin(min(self.pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY
        # TODO: on uphills this sometimes sets PERMIT_BRAKING low not considering the creep force
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch

        # GVC does not overshoot ego acceleration when starting from stop, but still has a similar delay
        if not self.CP.flags & ToyotaFlags.SECOC.value:
          a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo]))
        else:
          a_ego_blended = CS.out.aEgo

        # wind down integral when approaching target for step changes and smooth ramps to reduce overshoot
        prev_aego = self.aego.x
        self.aego.update(a_ego_blended)
        j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)

        future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
        a_ego_future = a_ego_blended + j_ego * future_t
        self.debug = a_ego_future

        self.debug2 = self.drv_filter(actuators.accel)
        print(actuators.accel, self.debug2)

        if CC.longActive:
          # # constantly slowly unwind integral to recover from large temporary errors
          # self.long_pid.i -= ACCEL_PID_UNWIND * float(np.sign(self.long_pid.i))
          #
          # error_future = pcm_accel_cmd - a_ego_future
          # pcm_accel_cmd = self.long_pid.update(error_future,
          #                                      speed=CS.out.vEgo,
          #                                      feedforward=pcm_accel_cmd,
          #                                      freeze_integrator=actuators.longControlState != LongCtrlState.pid)

          self.accel_filter.update(actuators.accel)
          self.accel_filter_slow.update(actuators.accel)

          self.accel_filter2.update(actuators.accel)
          self.accel_filter_slow2.update(actuators.accel)

          highpass_accel = self.accel_filter.x - self.accel_filter_slow.x
          highpass_accel2 = self.accel_filter2.x - self.accel_filter_slow2.x
          pcm_accel_cmd = actuators.accel - highpass_accel + highpass_accel2
          self.debug3 = pcm_accel_cmd
          self.debug4 = highpass_accel
          self.debug5 = highpass_accel2
          print(highpass_accel)

          # compensate for changes in pitch
          high_pass_pitch = self.pitch.x - self.pitch_slow.x
          pitch_compensation = float(np.clip(math.sin(high_pass_pitch) * ACCELERATION_DUE_TO_GRAVITY * 1.5, -1.5, 1.5))
          pcm_accel_cmd += pitch_compensation

        else:
          self.long_pid.reset()
          self.accel_filter.x = 0.0
          self.accel_filter_slow.x = 0.0
          self.accel_filter2.x = 0.0
          self.accel_filter_slow2.x = 0.0

        # Along with rate limiting positive jerk above, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min < 0.2 or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min > 0.3:
          self.permit_braking = False

        pcm_accel_cmd = float(np.clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

        can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.permit_braking, self.standstill_req, lead,
                                                        CS.acc_type, fcw_alert, self.distance_button))
        self.accel = pcm_accel_cmd

    else:
      # we can spam can to cancel the system even if we are using lat only control
      if pcm_cancel_cmd:
        if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button))

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    if self.CP.enableDsu:
      for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
        if self.frame % fr_step == 0 and self.CP.carFingerprint in cars:
          can_sends.append(CanData(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel
    new_actuators.debug = self.debug
    new_actuators.debug2 = self.debug2
    new_actuators.debug3 = self.debug3
    new_actuators.debug4 = self.debug4
    new_actuators.debug5 = self.debug5

    self.frame += 1
    return new_actuators, can_sends
