#!/usr/bin/env python3
import numpy as np
import random
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.lateral import MAX_LATERAL_ACCEL, MAX_LATERAL_JERK
from opendbc.car.ford.values import FordSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety

MSG_EngBrakeData = 0x165           # RX from PCM, for driver brake pedal and cruise state
MSG_EngVehicleSpThrottle = 0x204   # RX from PCM, for driver throttle input
MSG_BrakeSysFeatures = 0x415       # RX from ABS, for vehicle speed
MSG_EngVehicleSpThrottle2 = 0x202  # RX from PCM, for second vehicle speed
MSG_Yaw_Data_FD1 = 0x91            # RX from RCM, for yaw rate
MSG_Steering_Data_FD1 = 0x083      # TX by OP, various driver switches and LKAS/CC buttons
MSG_ACCDATA = 0x186                # TX by OP, ACC controls
MSG_ACCDATA_3 = 0x18A              # TX by OP, ACC/TJA user interface
MSG_Lane_Assist_Data1 = 0x3CA      # TX by OP, Lane Keep Assist
MSG_LateralMotionControl = 0x3D3   # TX by OP, Lateral Control message
MSG_LateralMotionControl2 = 0x3D6  # TX by OP, alternate Lateral Control message
MSG_IPMA_Data = 0x3D8              # TX by OP, IPMA and LKAS user interface


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr == MSG_Yaw_Data_FD1:
    chksum = dat[0] + dat[1]  # VehRol_W_Actl
    chksum += dat[2] + dat[3]  # VehYaw_W_Actl
    chksum += dat[5]  # VehRollYaw_No_Cnt
    chksum += dat[6] >> 6  # VehRolWActl_D_Qf
    chksum += (dat[6] >> 4) & 0x3  # VehYawWActl_D_Qf
    chksum = 0xff - (chksum & 0xff)
    ret[4] = chksum

  elif addr == MSG_BrakeSysFeatures:
    chksum = dat[0] + dat[1]  # Veh_V_ActlBrk
    chksum += (dat[2] >> 2) & 0xf  # VehVActlBrk_No_Cnt
    chksum += dat[2] >> 6  # VehVActlBrk_D_Qf
    chksum = 0xff - (chksum & 0xff)
    ret[3] = chksum

  elif addr == MSG_EngVehicleSpThrottle2:
    chksum = (dat[2] >> 3) & 0xf  # VehVActlEng_No_Cnt
    chksum += (dat[4] >> 5) & 0x3  # VehVActlEng_D_Qf
    chksum += dat[6] + dat[7]  # Veh_V_ActlEng
    chksum = 0xff - (chksum & 0xff)
    ret[1] = chksum

  return addr, ret, bus


class Buttons:
  CANCEL = 0
  RESUME = 1
  TJA_TOGGLE = 2


# Ford safety has four different configurations tested here:
#  * CAN with openpilot longitudinal
#  * CAN FD with stock longitudinal
#  * CAN FD with openpilot longitudinal

class TestFordSafetyBase(common.CarSafetyTest):
  STANDSTILL_THRESHOLD = 1
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_LateralMotionControl2, MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_LateralMotionControl2, MSG_IPMA_Data]}

  STEER_MESSAGE = 0

  # Curvature control limits
  DEG_TO_CAN = 50000  # 1 / (2e-5) rad to can
  MAX_CURVATURE = 0.02
  MAX_CURVATURE_ERROR = 0.002         # rad/m, = 100 CAN units
  CURVATURE_ERROR_MIN_SPEED = 10.0    # m/s

  # Frequency of the lateral control message, used for RT rate limit gating
  LATERAL_FREQUENCY = 20  # Hz

  cnt_speed = 0
  cnt_speed_2 = 0
  cnt_yaw_rate = 0
  cnt_lat_ctl = 0

  packer: CANPackerSafety
  safety: libsafety_py.LibSafety

  def _max_curvature_can(self, speed):
    # Mirrors C-side ISO 11270 lateral accel cap:
    #   fudged_speed = max(speed - 1, 1)
    #   max_curvature = MAX_LATERAL_ACCEL / fudged_speed^2
    #   max_curvature_can = int(max_curvature * curvature_to_can) + 1
    fudged_speed = max(speed - 1.0, 1.0)
    return int(MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed) * self.DEG_TO_CAN) + 1

  def _max_curvature_delta_can(self, speed):
    # Mirrors C-side lateral jerk cap (3.0 + bank ~= 3.6 m/s^3):
    #   fudged_speed = max(speed - 1, 1)
    #   max_curvature_rate_sec = MAX_LATERAL_JERK / fudged_speed^2
    #   max_curvature_delta_can = int(max_curvature_rate_sec / frequency * curvature_to_can) + 1
    fudged_speed = max(speed - 1.0, 1.0)
    max_curvature_rate_sec = MAX_LATERAL_JERK / (fudged_speed * fudged_speed)
    return int(max_curvature_rate_sec / self.LATERAL_FREQUENCY * self.DEG_TO_CAN) + 1

  def _set_prev_desired_angle(self, t):
    t = round(t * self.DEG_TO_CAN)
    self.safety.set_desired_curvature_last(t)

  def _reset_curvature_measurement(self, curvature, speed):
    for _ in range(6):
      self._rx(self._speed_msg(speed))
      self._rx(self._yaw_rate_msg(curvature, speed))

  # Driver brake pedal
  def _user_brake_msg(self, brake: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    enable = self.safety.get_controls_allowed()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # ABS vehicle speed
  def _speed_msg(self, speed: float, quality_flag=True):
    values = {"Veh_V_ActlBrk": speed * 3.6, "VehVActlBrk_D_Qf": 3 if quality_flag else 0, "VehVActlBrk_No_Cnt": self.cnt_speed % 16}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("BrakeSysFeatures", 0, values, fix_checksum=checksum)

  # PCM vehicle speed
  def _speed_msg_2(self, speed: float, quality_flag=True):
    # Ford relies on speed for driver curvature limiting, so it checks two sources
    values = {"Veh_V_ActlEng": speed * 3.6, "VehVActlEng_D_Qf": 3 if quality_flag else 0, "VehVActlEng_No_Cnt": self.cnt_speed_2 % 16}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle2", 0, values, fix_checksum=checksum)

  # Standstill state
  def _vehicle_moving_msg(self, speed: float):
    values = {"VehStop_D_Stat": 1 if speed <= self.STANDSTILL_THRESHOLD else random.choice((0, 2, 3))}
    return self.packer.make_can_msg_safety("DesiredTorqBrk", 0, values)

  # Current curvature
  def _yaw_rate_msg(self, curvature: float, speed: float, quality_flag=True):
    values = {"VehYaw_W_Actl": curvature * speed, "VehYawWActl_D_Qf": 3 if quality_flag else 0,
              "VehRollYaw_No_Cnt": self.cnt_yaw_rate % 256}
    self.__class__.cnt_yaw_rate += 1
    return self.packer.make_can_msg_safety("Yaw_Data_FD1", 0, values, fix_checksum=checksum)

  # Drive throttle input
  def _user_gas_msg(self, gas: float):
    values = {"ApedPos_Pc_ActlArb": gas}
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle", 0, values)

  # Cruise status
  def _pcm_status_msg(self, enable: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    brake = self.safety.get_brake_pressed_prev()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # LKAS command
  def _lkas_command_msg(self, action: int):
    values = {
      "LkaActvStats_D2_Req": action,
    }
    return self.packer.make_can_msg_safety("Lane_Assist_Data1", 0, values)

  # LCA command
  def _lat_ctl_msg(self, enabled: bool, path_offset: float, path_angle: float, curvature: float, curvature_rate: float,
                   increment_timer: bool = True):
    # Advance the safety's internal timer so the RT rate limit (rt_angle_rate_limit_check)
    # doesn't trip during long test loops. Matches the lateral message frequency.
    if increment_timer:
      self.safety.set_timer(self.cnt_lat_ctl * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_lat_ctl += 1
    if self.STEER_MESSAGE == MSG_LateralMotionControl:
      values = {
        "LatCtl_D_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCurv_NoRate_Actl": curvature_rate,  # Curvature rate [-0.001024|0.00102375] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl", 0, values)
    elif self.STEER_MESSAGE == MSG_LateralMotionControl2:
      values = {
        "LatCtl_D2_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCrv_NoRate2_Actl": curvature_rate,  # Curvature rate [-0.001024|0.001023] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl2", 0, values)

  # Cruise control buttons
  def _acc_button_msg(self, button: int, bus: int):
    values = {
      "CcAslButtnCnclPress": 1 if button == Buttons.CANCEL else 0,
      "CcAsllButtnResPress": 1 if button == Buttons.RESUME else 0,
      "TjaButtnOnOffPress": 1 if button == Buttons.TJA_TOGGLE else 0,
    }
    return self.packer.make_can_msg_safety("Steering_Data_FD1", bus, values)

  def test_rx_hook(self):
    # checksum, counter, and quality flag checks
    for quality_flag in [True, False]:
      for msg_type in ["speed", "speed_2", "yaw"]:
        self.safety.set_controls_allowed(True)
        # send multiple times to verify counter checks
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)
          elif msg_type == "yaw":
            msg = self._yaw_rate_msg(0, 0, quality_flag=quality_flag)

          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # Mess with checksum to make it fail, checksum is not checked for 2nd speed
        msg[0].data[3] = 0  # Speed checksum & half of yaw signal
        should_rx = msg_type == "speed_2" and quality_flag
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_angle_measurements(self):
    """Tests rx hook correctly parses the curvature measurement from the vehicle speed and yaw rate"""
    for speed in np.arange(0.5, 40, 0.5):
      for curvature in np.arange(0, self.MAX_CURVATURE * 2, 2e-3):
        self._rx(self._speed_msg(speed))
        for c in (curvature, -curvature, 0, 0, 0, 0):
          self._rx(self._yaw_rate_msg(c, speed))

        self.assertEqual(self.safety.get_curvature_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_curvature_meas_max(), round(curvature * self.DEG_TO_CAN))

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_curvature_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_curvature_meas_max(), 0)

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_curvature_meas_min(), 0)
        self.assertEqual(self.safety.get_curvature_meas_max(), 0)

  def test_max_lateral_acceleration(self):
    # ISO 11270 lateral acceleration cap (3.6 m/s^2 with bank tolerance) limits curvature based on speed.
    # The C-side check is `safety_max_limit_check(val, max_can, -max_can)` where
    # `max_can = int(MAX_LATERAL_ACCEL / max(speed - 1, 1)^2 * DEG_TO_CAN) + 1`.
    for speed in np.arange(0, 40, 0.5):
      max_can = self._max_curvature_can(speed)
      # Boundary samples in CAN units: deep below, just below, at, just above, deep above the limit.
      for offset in (-5, -1, 0, 1, 5):
        curvature_can = max_can + offset
        # Skip values exceeding the message-encodable range (|curvature| <= MAX_CURVATURE).
        if abs(curvature_can) > round(self.MAX_CURVATURE * self.DEG_TO_CAN):
          continue
        curvature = curvature_can / self.DEG_TO_CAN

        for sign in (-1, 1):
          signed_curvature = sign * curvature
          self.safety.set_controls_allowed(True)
          self._set_prev_desired_angle(signed_curvature)
          self._reset_curvature_measurement(signed_curvature, speed)

          should_tx = abs(curvature_can) <= max_can
          self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, signed_curvature, 0)),
                           msg=f"speed={speed} curvature_can={sign * curvature_can} max_can={max_can}")

  def test_curvature_jerk_rate_limit(self):
    # ISO 11270 lateral jerk cap (3.6 m/s^3 with bank tolerance) limits curvature delta per frame.
    # Per-frame delta from previous curvature must be <= max_curvature_delta_can.
    # The Ford CAN signal encodes curvature in [-0.02, 0.02094]; values outside that are clamped by
    # the DBC packer, so we restrict tests to speeds where the test curvatures fit on-wire.
    max_encoded_can = int(self.MAX_CURVATURE * self.DEG_TO_CAN)  # 1000
    for speed in np.arange(5, 40, 2.5):
      max_can = self._max_curvature_can(speed)
      max_delta_can = self._max_curvature_delta_can(speed)
      # Start from a small previous curvature well within the accel cap and with room to grow.
      base_can = max_delta_can * 2
      # Skip speeds where prev + delta would exceed the encodable signal range or the accel cap.
      if base_can + max_delta_can + 5 > min(max_can, max_encoded_can):
        continue

      for sign in (-1, 1):
        prev_can = sign * base_can
        # Test boundary offsets: just inside, at, and just outside the jerk limit.
        for offset in (-1, 0, 1, 5):
          # Up direction (away from 0): prev + sign*(max_delta + offset)
          desired_can = prev_can + sign * (max_delta_can + offset)
          self.safety.set_controls_allowed(True)
          self.safety.set_desired_curvature_last(prev_can)
          self._reset_curvature_measurement(prev_can / self.DEG_TO_CAN, speed)
          should_tx = offset <= 0
          self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, desired_can / self.DEG_TO_CAN, 0)),
                           msg=f"up: speed={speed} prev={prev_can} desired={desired_can} max_delta={max_delta_can}")

          # Down direction (toward 0 and through): prev - sign*(max_delta + offset)
          desired_can = prev_can - sign * (max_delta_can + offset)
          self.safety.set_controls_allowed(True)
          self.safety.set_desired_curvature_last(prev_can)
          self._reset_curvature_measurement(prev_can / self.DEG_TO_CAN, speed)
          should_tx = offset <= 0
          self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, desired_can / self.DEG_TO_CAN, 0)),
                           msg=f"down: speed={speed} prev={prev_can} desired={desired_can} max_delta={max_delta_can}")

  def test_curvature_violation_resets_prev(self):
    # After a violation OR when controls are not allowed, desired_curvature_last must reset to 0.
    # Verify by triggering a violation (controls not allowed), then confirming that a subsequent
    # non-zero command beyond the per-frame jerk limit (from 0) is rejected.
    speed = 25.
    max_delta_can = self._max_curvature_delta_can(speed)

    # First: drop controls_allowed and send a command -> safety should reset prev to 0.
    self.safety.set_controls_allowed(False)
    self.safety.set_desired_curvature_last(max_delta_can * 5)
    self._reset_curvature_measurement(0, speed)
    self._tx(self._lat_ctl_msg(False, 0, 0, 0, 0))  # inactive cmd, !controls_allowed -> reset prev to 0

    # Now re-enable controls. desired_curvature_last should be 0; jumping past jerk limit must fail.
    self.safety.set_controls_allowed(True)
    over_delta = (max_delta_can + 5) / self.DEG_TO_CAN
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, over_delta, 0)))

    # And jumping by exactly max_delta from 0 should pass.
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_curvature_last(0)
    at_delta = max_delta_can / self.DEG_TO_CAN
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, at_delta, 0)))

  def test_curvature_prev_tracks_after_pass(self):
    # After a non-violation tx with controls_allowed, desired_curvature_last must be set to the
    # commanded curvature (not reset to 0). This protects against a regression in the reset block
    # where the condition would erroneously reset on every controls_allowed tx.
    speed = 25.
    max_delta_can = self._max_curvature_delta_can(speed)
    # Pick a base curvature several jerk-limit deltas away from 0 so that, if prev were
    # incorrectly reset to 0, the next command would exceed the per-frame jerk limit.
    base_can = max_delta_can * 5

    # Step 1: walk up to base_can by setting prev directly, then send base_can (delta=0 -> passes).
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_curvature_last(base_can)
    self._reset_curvature_measurement(base_can / self.DEG_TO_CAN, speed)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, base_can / self.DEG_TO_CAN, 0)))

    # Step 2: next command at base_can + max_delta_can should pass (delta within jerk limit)
    # ONLY IF prev tracked correctly to base_can. If prev was reset to 0, delta would be
    # (base_can + max_delta_can) which is well beyond max_delta_can -> would fail.
    next_can = base_can + max_delta_can
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, next_can / self.DEG_TO_CAN, 0)),
                    msg=f"prev should track commanded curvature; speed={speed} base={base_can} next={next_can}")

  def test_curvature_error_clamps_to_meas(self):
    # Above curvature_error_min_speed, commanded curvature must be within max_curvature_error
    # of the measured curvature (with a +/-1 CAN-unit fudge). Tests both bounds.
    speed = 15.
    max_error_can = round(self.MAX_CURVATURE_ERROR * self.DEG_TO_CAN)  # 100
    self._reset_curvature_measurement(0, speed)  # meas.min == meas.max == 0

    # Upper bound: band is [-(max_error+1), +(max_error+1)] = [-101, 101]
    prev_can = max_error_can  # 100, stays within jerk limit of test cmds
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_curvature_last(prev_can)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, (max_error_can + 1) / self.DEG_TO_CAN, 0)))

    self.safety.set_controls_allowed(True)
    self.safety.set_desired_curvature_last(prev_can)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, (max_error_can + 2) / self.DEG_TO_CAN, 0)))

    # Lower bound: symmetric.
    prev_can = -max_error_can
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_curvature_last(prev_can)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, -(max_error_can + 1) / self.DEG_TO_CAN, 0)))

    self.safety.set_controls_allowed(True)
    self.safety.set_desired_curvature_last(prev_can)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, -(max_error_can + 2) / self.DEG_TO_CAN, 0)))

  def test_curvature_error_disabled_below_min_speed(self):
    # Below curvature_error_min_speed, the rack-tracking check is skipped: a command outside
    # the measured-curvature error band but within ISO accel + jerk caps must pass.
    speed = 9.  # < CURVATURE_ERROR_MIN_SPEED
    max_error_can = round(self.MAX_CURVATURE_ERROR * self.DEG_TO_CAN)
    self._reset_curvature_measurement(0, speed)

    prev_can = max_error_can
    cmd_can = max_error_can + 5  # outside [-101, 101], well inside ISO accel cap at speed=9
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_curvature_last(prev_can)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, cmd_can / self.DEG_TO_CAN, 0)))

  def test_curvature_error_uses_newest_speed(self):
    # The check must read vehicle_speed.values[0] (newest). Stage the speed buffer so values[0]
    # crosses the min_speed threshold while older samples don't: original enforces, a mutant
    # reading any older index would skip.
    max_error_can = round(self.MAX_CURVATURE_ERROR * self.DEG_TO_CAN)
    self._reset_curvature_measurement(0, 9)  # all speed samples = 9, meas = 0
    self._rx(self._speed_msg(15))  # values[0] = 15, values[1..5] = 9

    prev_can = max_error_can
    cmd_can = max_error_can + 5  # outside band; jerk delta = 5, well within limits
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_curvature_last(prev_can)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, cmd_can / self.DEG_TO_CAN, 0)))

  def test_steer_allowed(self):
    # New simple curvature safety:
    #   * path_offset, path_angle, curvature_rate must be 0 (Ford message uses inactive sentinels)
    #   * when steer_control_enabled is True: curvature within ISO lateral accel cap, controls_allowed required
    #   * when steer_control_enabled is False: curvature must be exactly 0
    path_offsets = np.arange(-5.12, 5.11, 2.5).round()
    path_angles = np.arange(-0.5, 0.5235, 0.25).round(1)
    curvature_rates = np.arange(-0.001024, 0.00102375, 0.001).round(3)
    curvatures = np.arange(-0.02, 0.02094, 0.01).round(2)

    for speed in (5., 25.):
      max_can = self._max_curvature_can(speed)
      for controls_allowed in (True, False):
        for steer_control_enabled in (True, False):
          for path_offset in path_offsets:
            for path_angle in path_angles:
              for curvature_rate in curvature_rates:
                for curvature in curvatures:
                  self.safety.set_controls_allowed(controls_allowed)
                  # prev curvature == desired -> jerk rate check trivially satisfied
                  self._set_prev_desired_angle(curvature)
                  self._reset_curvature_measurement(curvature, speed)

                  # other lateral signals must be at their inactive sentinel
                  should_tx = path_offset == 0 and path_angle == 0 and curvature_rate == 0

                  if steer_control_enabled:
                    # controls must be allowed and curvature within ISO accel cap
                    curvature_can = round(curvature * self.DEG_TO_CAN)
                    should_tx = should_tx and controls_allowed and abs(curvature_can) <= max_can
                  else:
                    # when disabled, curvature must be exactly zero (independent of controls_allowed)
                    should_tx = should_tx and curvature == 0

                  with self.subTest(controls_allowed=controls_allowed, steer_control_enabled=steer_control_enabled,
                                    path_offset=float(path_offset), path_angle=float(path_angle),
                                    curvature_rate=float(curvature_rate), curvature=float(curvature), speed=float(speed)):
                    self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(steer_control_enabled, path_offset, path_angle,
                                                                           curvature, curvature_rate)))

  def test_prevent_lkas_action(self):
    self.safety.set_controls_allowed(1)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

  def test_acc_buttons(self):
    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for enabled in (True, False):
        self._rx(self._pcm_status_msg(enabled))
        self.assertTrue(self._tx(self._acc_button_msg(Buttons.TJA_TOGGLE, 2)))

    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for bus in (0, 2):
        self.assertEqual(allowed, self._tx(self._acc_button_msg(Buttons.RESUME, bus)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      for bus in (0, 2):
        self.assertEqual(enabled, self._tx(self._acc_button_msg(Buttons.CANCEL, bus)))


class TestFordCANFDStockSafety(TestFordSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.CANFD)
    self.safety.init_tests()


class TestFordLongitudinalSafetyBase(TestFordSafetyBase):
  MAX_ACCEL = 2.0  # accel is used for brakes, but openpilot can set positive values
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  MAX_GAS = 2.0
  MIN_GAS = -0.5
  INACTIVE_GAS = -5.0

  # ACC command
  def _acc_command_msg(self, gas: float, brake: float, brake_actuation: bool, cmbb_deny: bool = False):
    values = {
      "AccPrpl_A_Rq": gas,                              # [-5|5.23] m/s^2
      "AccPrpl_A_Pred": gas,                            # [-5|5.23] m/s^2
      "AccBrkTot_A_Rq": brake,                          # [-20|11.9449] m/s^2
      "AccBrkPrchg_B_Rq": 1 if brake_actuation else 0,  # Pre-charge brake request: 0=No, 1=Yes
      "AccBrkDecel_B_Rq": 1 if brake_actuation else 0,  # Deceleration request: 0=Inactive, 1=Active
      "CmbbDeny_B_Actl": 1 if cmbb_deny else 0,         # [0|1] deny AEB actuation
    }
    return self.packer.make_can_msg_safety("ACCDATA", 0, values)

  def test_stock_aeb(self):
    # Test that CmbbDeny_B_Actl is never 1, it prevents the ABS module from actuating AEB requests from ACCDATA_2
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for cmbb_deny in (True, False):
        should_tx = not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, self.INACTIVE_ACCEL, controls_allowed, cmbb_deny)))
        should_tx = controls_allowed and not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.MAX_GAS, self.MAX_ACCEL, controls_allowed, cmbb_deny)))

  def test_gas_safety_check(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for gas in np.concatenate((np.arange(self.MIN_GAS - 2, self.MAX_GAS + 2, 0.05), [self.INACTIVE_GAS])):
        gas = round(gas, 2)  # floats might not hit exact boundary conditions without rounding
        should_tx = (controls_allowed and self.MIN_GAS <= gas <= self.MAX_GAS) or gas == self.INACTIVE_GAS
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(gas, self.INACTIVE_ACCEL, controls_allowed)))

  def test_brake_safety_check(self):
    brake_values = self._boundary_values([self.MIN_ACCEL, self.MAX_ACCEL, self.INACTIVE_ACCEL],
                                         self.MIN_ACCEL - 2, self.MAX_ACCEL + 2, 0.05)
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for brake_actuation in (True, False):
        for brake in brake_values:
          should_tx = (controls_allowed and self.MIN_ACCEL <= brake <= self.MAX_ACCEL) or brake == self.INACTIVE_ACCEL
          should_tx = should_tx and (controls_allowed or not brake_actuation)
          self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, brake, brake_actuation)))


class TestFordLongitudinalSafety(TestFordLongitudinalSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    # Make sure we enforce long safety even without long flag for CAN
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, 0)
    self.safety.init_tests()


class TestFordCANFDLongitudinalSafety(TestFordLongitudinalSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD)
    self.safety.init_tests()


if __name__ == "__main__":
  unittest.main()
