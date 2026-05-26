import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.ford import fordcan
from opendbc.car.ford.lateral_bal import FordBalLiveScale, bal_encode
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# CAN FD limits:
# Limit to average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll raises lateral acceleration
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL - (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~2.4 m/s^2

# Path polynomial DBC clip magnitudes — match LateralMotionControl2 signal ranges
FORD_PATH_C0_CLIP = (-5.12, 5.11)     # m
FORD_PATH_C1_CLIP = (-0.5, 0.5235)    # rad
FORD_PATH_C2_CLIP = (-0.02, 0.02)     # 1/m (the DBC allows +0.02094 but bal stays symmetric)
FORD_CURVATURE_RATE_CLIP = 0.001023   # 1/m² — LatCtlCrv_NoRate2_Actl


def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5  # 5s smooths over the overshoot
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))


def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, steering_angle, lat_active, CP):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, steering_angle, lat_active, CarControllerParams.ANGLE_LIMITS)

  # Ford Q4/CAN FD has more torque available compared to Q3/CAN so we limit it based on lateral acceleration.
  # Safety is not aware of the road roll so we subtract a conservative amount at all times
  if CP.flags & FordFlags.CANFD:
    # Limit curvature to conservative max lateral acceleration
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))

  return apply_curvature


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.apply_curvature_last = 0
    self.anti_overshoot_curvature_last = 0
    self.desired_curvature_last = 0.0
    self.path_angle_last = 0.0
    self.path_offset_last = 0.0
    # bal: PSCM-inverse encoder with always-on live scale estimator.
    # Soft-imports openpilot.common.params for persistence; runs in-memory
    # when Params isn't available (e.g. standalone opendbc tests).
    self.bal_live_scale = FordBalLiveScale()

    self.accel = 0.0
    self.gas = 0.0
    self.brake_request = False

    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl
    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### lateral control ###
    # send steer msg at 20Hz
    apply_curvature = 0.0
    path_angle = 0.0
    path_offset = 0.0
    curvature_rate = 0.0
    ramp_type = 0

    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      desired_curvature = 0.0

      if CC.latActive:
        v_ego = CS.out.vEgoRaw
        desired_curvature = actuators.curvature

        # Bronco and some other cars consistently overshoot curv requests.
        # Apply the same input shaping before either Ford lateral command path.
        if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14):
          self.anti_overshoot_curvature_last = anti_overshoot(desired_curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
          desired_curvature = self.anti_overshoot_curvature_last

        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

        if self.CP.flags & FordFlags.CANFD:
          # bal: PSCM-inverse polynomial encoding. The live scale estimator
          # (always-on, bounded ±15% from the platform default) adapts a
          # per-vehicle multiplier on the regression coefficients.
          fingerprint = str(self.CP.carFingerprint)
          act_k = -CS.out.yawRate / max(v_ego, 0.5)
          self.bal_live_scale.update(desired_curvature, act_k, v_ego,
                                     CC.latActive, CS.out.steeringPressed)
          live = self.bal_live_scale.current_scale(fingerprint)

          c0_int, c1_int, c2_int = bal_encode(desired_curvature, v_ego, fingerprint, live)

          path_offset    = float(np.clip(c0_int, FORD_PATH_C0_CLIP[0], FORD_PATH_C0_CLIP[1]))
          path_angle     = float(np.clip(c1_int, FORD_PATH_C1_CLIP[0], FORD_PATH_C1_CLIP[1]))
          apply_curvature = float(np.clip(c2_int, FORD_PATH_C2_CLIP[0], FORD_PATH_C2_CLIP[1]))

          # Curvature rate (4th polynomial slot, LatCtlCrv_NoRate2_Actl):
          # dκ/dx in 1/m². PSCM expects per-distance so dκ/dt is divided
          # by v_ego. Frame-to-frame derivative on raw planner desk.
          dt = DT_CTRL * CarControllerParams.STEER_STEP
          desk_dot_per_meter = (desired_curvature - self.desired_curvature_last) / (dt * max(v_ego, 1.0))
          curvature_rate = float(np.clip(desk_dot_per_meter,
                                          -FORD_CURVATURE_RATE_CLIP,
                                           FORD_CURVATURE_RATE_CLIP))
          ramp_type = 3
        else:
          # Non-CAN FD: curvature-only control (unchanged from upstream)
          apply_curvature = desired_curvature
          apply_curvature = apply_ford_curvature_limits(apply_curvature, self.apply_curvature_last, current_curvature,
                                                        CS.out.vEgoRaw, 0., CC.latActive, self.CP)

      self.path_angle_last = path_angle
      self.path_offset_last = path_offset
      self.apply_curvature_last = apply_curvature
      self.desired_curvature_last = desired_curvature

      if self.CP.flags & FordFlags.CANFD:
        mode = 2 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(
          self.packer, self.CAN, mode, ramp_type, 1, -path_offset, -path_angle,
          -apply_curvature, -curvature_rate, counter
        ))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(
          self.packer, self.CAN, CC.latActive, 0., 0., -self.apply_curvature_last, 0.
        ))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = actuators.accel
      gas = accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction is very slow
        # TODO: verify this applies to EV/hybrid
        accel = apply_creep_compensation(accel, CS.out.vEgo)

        # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
        # however even 3.5 m/s^3 causes some overshoot with a step response.
        accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Both gas and accel are in m/s^2, accel is used solely for braking
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      accel_pitch_compensated = accel + accel_due_to_pitch
      if accel_pitch_compensated > 0.3 or not CC.longActive:
        self.brake_request = False
      elif accel_pitch_compensated < 0.0:
        self.brake_request = True

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      # TODO: look into using the actuators packet to send the desired speed
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, self.brake_request, v_ego_kph=V_CRUISE_MAX))

      self.accel = accel
      self.gas = gas

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      self.distance_bar_frame = self.frame

    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      show_distance_bars = self.frame - self.distance_bar_frame < 400
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, show_distance_bars,
                                                 hud_control, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
