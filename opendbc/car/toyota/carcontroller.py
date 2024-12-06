import math
from collections import deque

from opendbc.car import Bus, carlog, apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                        make_tester_present_msg, rate_limit, structs, ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.common.numpy_fast import clip, interp
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

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# LTA limits
# EPS ignores commands above this angle and causes PCS to fault
MAX_LTA_ANGLE = 94.9461  # deg
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes


def get_long_tune(CP, params):
  kiBP = [0.]
  kdBP = [0.]
  kdV = [0.]
  if CP.carFingerprint in TSS2_CAR:
    kiV = [0.25]
    kdV = [0.0 / 4]

    # Since we compensate for imprecise acceleration in carcontroller and error correct on aEgo, we can avoid using gains
    if CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT:
      kiV = [0.0]
  else:
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]

  return PIDController(0.0, (kiBP, kiV), k_f=1.0, k_d=(kdBP, kdV),
                       pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
                       rate=1 / (DT_CTRL * 3))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.distance_button = 0

    self.aego_d = deque([0.0] * 100, maxlen=100)

    self.long_pid = get_long_tune(self.CP, self.params)

    self.error_rate = FirstOrderFilter(0.0, 0.5, DT_CTRL * 3)
    self.error_rate2 = FirstOrderFilter(0.0, 1, DT_CTRL * 3)
    self.error_rate3 = FirstOrderFilter(0.0, 0.75, DT_CTRL)
    self.error_rate4 = FirstOrderFilter(0.0, 0.15, DT_CTRL)
    self.d = deque([0.0] * 100, maxlen=100)
    self.prev_error = 0.0
    self.prev_error2 = 0.0

    self.f = FirstOrderFilter(0.0, 0.4, DT_CTRL * 3)
    self.f2 = FirstOrderFilter(0.0, 0.8, DT_CTRL * 3)

    self.pcm_accel_cmd = FirstOrderFilter(0, 0.05, DT_CTRL * 3)
    self.pcm_accel_cmd_d = deque([0.0] * 100, maxlen=100)

    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL)

    # *** start PCM compensation state ***
    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)

    # FIXME: rate isn't set properly
    self.pcm_pid = PIDController(2.0, 0.5, 1 / (DT_CTRL * 3))
    self.pcm_accel_compensation = FirstOrderFilter(0, 0.5, DT_CTRL * 3)

    # the PCM's reported acceleration request can sometimes mismatch aEgo, close the loop
    self.pcm_accel_net_offset = FirstOrderFilter(0, 1.0, DT_CTRL * 3)

    # aEgo also often lags behind the PCM request due to physical brake lag which varies by car,
    # so we error correct on the filtered PCM acceleration request using the actuator delay.
    # TODO: move the delay into the interface
    self.pcm_accel_net = FirstOrderFilter(0, self.CP.longitudinalActuatorDelay, DT_CTRL * 3)
    self.net_acceleration_request = FirstOrderFilter(0, 0.15, DT_CTRL * 3)
    if not any(fw.ecu == Ecu.hybrid for fw in self.CP.carFw):
      self.pcm_accel_net.update_alpha(self.CP.longitudinalActuatorDelay + 0.2)
      self.net_acceleration_request.update_alpha(self.CP.longitudinalActuatorDelay + 0.2)
    # *** end PCM compensation state ***

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.accel = 0
    self.prev_accel = 0

    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_prev_reset_counter = 0
    self.secoc_mismatch_counter = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])

    # *** control msgs ***
    can_sends = []

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        expected_mac = build_sync_mac(self.secoc_key, int(CS.secoc_synchronization['TRIP_CNT']), int(CS.secoc_synchronization['RESET_CNT']))
        if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac and self.secoc_mismatch_counter < 100:
          carlog.error("SecOC synchronization MAC mismatch, wrong key?")
          self.secoc_mismatch_counter += 1

    # *** steer torque ***
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_steer = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)

        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

    self.last_steer = apply_steer

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    steer_command = toyotacan.create_steer_command(self.packer, apply_steer, apply_steer_req)
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
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

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

    # GVC does not overshoot ego acceleration when starting from stop, but still has a similar delay
    if not self.CP.flags & ToyotaFlags.SECOC.value:
      a_ego_blended = interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo])
    else:
      a_ego_blended = CS.out.aEgo

    # TODO: this can be moved in the 33hz loop with no side-effects
    prev_aego = self.aego.x
    self.aego.update(a_ego_blended)
    jEgo = (self.aego.x - prev_aego) / DT_CTRL
    # TODO: adjust for hybrid
    future_aego = a_ego_blended + jEgo * 0.5

    # self.debug = self.aego.x + jEgo * 0.5


    jEgo = (a_ego_blended - self.aego_d[-15]) / (DT_CTRL * 15)
    future_aego2 = a_ego_blended + jEgo * 0.5
    self.aego_d.append(a_ego_blended)

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

        # calculate amount of acceleration PCM should apply to reach target, given pitch
        accel_due_to_pitch = math.sin(self.pitch.x) * ACCELERATION_DUE_TO_GRAVITY
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch
        self.net_acceleration_request.update(net_acceleration_request)

        # For cars where we allow a higher max acceleration of 2.0 m/s^2, compensate for PCM request overshoot and imprecise braking
        if self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT and CC.longActive and not CS.out.cruiseState.standstill:
          # filter ACCEL_NET so it more closely matches aEgo delay for error correction
          self.pcm_accel_net.update(CS.pcm_accel_net)

          # Our model of the PCM's acceleration request isn't perfect, so we learn the offset when moving
          new_pcm_accel_net = CS.pcm_accel_net
          if stopping or CS.out.standstill:
            # TODO: check if maintaining the offset from before stopping is beneficial
            self.pcm_accel_net_offset.x = 0.0
          else:
            new_pcm_accel_net -= self.pcm_accel_net_offset.update((self.pcm_accel_net.x - accel_due_to_pitch) - CS.out.aEgo)

          # let PCM handle stopping for now, error correct on a delayed acceleration request
          pcm_accel_compensation = 0.0
          if not stopping:
            # prevent compensation windup
            self.pcm_pid.neg_limit = pcm_accel_cmd - self.params.ACCEL_MAX
            self.pcm_pid.pos_limit = pcm_accel_cmd - self.params.ACCEL_MIN
            pcm_accel_compensation = self.pcm_pid.update(new_pcm_accel_net - self.net_acceleration_request.x)
          else:
            self.pcm_pid.reset()

          pcm_accel_cmd = pcm_accel_cmd - self.pcm_accel_compensation.update(pcm_accel_compensation)

        else:
          self.pcm_accel_compensation.x = 0.0
          self.pcm_accel_net_offset.x = 0.0
          self.net_acceleration_request.x = 0.0
          self.pcm_accel_net.x = CS.pcm_accel_net
          self.pcm_pid.reset()
          self.permit_braking = True

        if not (self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT):
          if actuators.longControlState == LongCtrlState.pid:
            error = pcm_accel_cmd - a_ego_blended
            self.error_rate.update((error - self.prev_error) / (DT_CTRL * 3))
            self.error_rate2.update((error - self.prev_error) / (DT_CTRL * 3))
            # self.error_rate.x = (error - self.prev_error) / (DT_CTRL * 3)
            print(error, self.prev_error, self.error_rate.x)
            self.prev_error = error

            error = pcm_accel_cmd - future_aego
            pcm_accel_cmd = self.long_pid.update(error, error_rate=self.error_rate4.x,  # self.error_rate.x,
                                                 speed=CS.out.vEgo,
                                                 override=abs(error) > 0.5,
                                                 feedforward=pcm_accel_cmd)

            self.f.update(pcm_accel_cmd)
            self.f2.update(pcm_accel_cmd)
            # self.debug = self.f.x - self.f2.x

            prev_pcm_accel_cmd = pcm_accel_cmd
            self.pcm_accel_cmd_d.append(pcm_accel_cmd)
            self.pcm_accel_cmd.update(pcm_accel_cmd)

            pcm_accel_cmd_jerk = (pcm_accel_cmd - prev_pcm_accel_cmd) / (DT_CTRL * 3)
            # pcm_accel_cmd_jerk = (self.pcm_accel_cmd_d[-1] - self.pcm_accel_cmd_d[-15 // 3]) / (DT_CTRL * 15)
            pcm_accel_cmd_future = pcm_accel_cmd + pcm_accel_cmd_jerk * 0.5

            self.debug2 = self.f.x - pcm_accel_cmd_future


            pcm_accel_cmd -= (self.debug2)

          else:
            self.long_pid.reset()
            self.error_rate.x = 0.0
            self.error_rate2.x = 0.0
            self.prev_error = 0.0
            self.f.x = 0
            self.f2.x = 0
            self.pcm_accel_cmd.x = 0

        error = pcm_accel_cmd - a_ego_blended
        # self.error_rate3.update((error - self.prev_error2) / (DT_CTRL))
        # self.debug3 = self.error_rate3.x
        self.debug3 = (error - self.d[-15 // 3]) / (DT_CTRL * 15)
        self.debug3 = self.error_rate4.update(self.debug3)
        self.prev_error2 = error
        self.d.append(error)

        self.debug = future_aego
        self.debug2 = future_aego2

        # Along with rate limiting positive jerk above, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min < 0.2 or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min > 0.3:
          self.permit_braking = False

        pcm_accel_cmd = clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

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
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(CanData(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    # pcm_accel_cmd = actuators.accel
    # error = pcm_accel_cmd - a_ego_blended
    # self.error_rate3.update((error - self.prev_error2) / (DT_CTRL))
    # self.d.append(error)
    # # self.debug3 = self.error_rate3.x
    # if self.frame % 3 == 0:
    #   self.debug3 = (error - self.d[-10]) / (DT_CTRL * 10)
    #   # self.debug3 = self.error_rate4.update(self.debug3)
    # self.prev_error2 = error

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    new_actuators.debug = self.debug
    new_actuators.debug2 = self.debug2
    new_actuators.debug3 = self.debug3

    self.frame += 1
    return new_actuators, can_sends
