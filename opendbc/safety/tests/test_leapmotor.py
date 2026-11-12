#!/usr/bin/env python3
import math
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.leapmotor.carcontroller import get_safety_CP
from opendbc.car.leapmotor.values import CarControllerParams
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed

ADAS_LKA = 0x83
POWERTRAIN_STATE = 0xB7


def round_angle(apply_angle, can_offset=0):
  # STEER_ANGLE_REQ is -0.02 deg per bit, offset 649.8 deg, so 0 deg is exactly representable (raw 32490).
  # can_offset is in angle units of 0.02 deg (positive = larger angle)
  raw = math.floor((apply_angle - 649.8) / -0.02 + 0.5) - can_offset
  return raw * -0.02 + 649.8

REAL_FRAMES = {  # 20 consecutive frames per message from a 2025 global B10, route 00000232 (driving with ACC/LCC)
  0x184: [
    "14044E1E7D1EF41D061E55201CF3004FEB02CE59A8DE808888000800000000F0",
    "BF056B1E881EF41D021E55601D063150FE021C2678DF878888000800000000F0",
    "06066E1E801EF81DFB1D55901E1A615112030302C8217C8888000800000000F0",
    "2607521E791E111EED1D55D01F2DA1522503EAF597E0798888000800000000F0",
    "AA08671E801E0A1EFB1D55002141D153380310EA479E778888000800000000F0",
    "AB098F1E791EF41D111E5540225401554B03065A08DF808888000800000000F0",
    "6D0A601E791EFF1D1F1E5570236831565F03D41998A18C8888000800000000F0",
    "B40B591E961E0D1EE21D55B0247B6157720313EA27E2758888000800000000F0",
    "B40C801E921EFB1DED1D55E0258FA1588503221AA85E7F8888000800000000F0",
    "5D0D841E8B1E061E0A1E552027A2D159980303C2D79E778888000800000000F0",
    "8B0E791E8B1E061EFF1D555028B6015BAB03002638E07D8888000800000000F0",
    "0A0F6E1E881EF01D0D1E559029CA315CBF03ED01481F878888000800000000F0",
    "9300641E8F1EED1DF41D55C02ADD615DD203FAF5C7A2778888000800000000F0",
    "B301761E8F1E111EFF1D55002CF1A15EE503133638617F8888000800000000F0",
    "B802791E801E0A1EFB1D55302D04D25FF80309D2E71D758888000800000000F0",
    "7C03761E791EE91D021E55702E1802610B040926B85D838888000800000000F0",
    "B104591E791EED1DFB1D55A02F2B32621F04E70DA81F848888000800000000F0",
    "4505591E801EE91DF41D55E0303F6263320413F6C7217D8888000800000000F0",
    "1406671E761E0A1EF81D55103252A26445040DDE67E0798888000800000000F0",
    "F0076E1E641E061EF81D55503366D2655804FD7947DE768888000800000000F0",
  ],
  0x18D: [
    "300A02717E01F7438EAF4C0104030000F6018807A3D403000000000000000000",
    "4F0B026E7E00F5438E834C0104040000F4018808A3D503000000000000000000",
    "440C026E7E00F5438E8B4C0103040000F5018808A3D303000000000000000000",
    "490D026B7E00F8438EB24C0104030000F7018807A3D403000000000000000000",
    "A40E026B7E00FB438E984C0103040000F5018808A3D203000000000000000000",
    "A80F02687E00F9438E924C0104040000F5018808A3D203000000000000000000",
    "B90002687E01F6438E634C0104040000F3018809A3D303000000000000000000",
    "530102657E00F6438E9A4C0104040000F5018808A3D003000000000000000000",
    "840202627E00FB438EA34C0103040000F6018808A3D403000000000000000000",
    "A20302627E00FC438E694C0104040000F3018809A3CE03000000000000000000",
    "2704025E7E00F8438E6F4C0104040000F3018809A3D103000000000000000000",
    "EB05025E7E00F3438E2F4C0105050000F001880AA3CF03000000000000000000",
    "A606025B7E01F2438E4C4C0104040000F2018809A3D303000000000000000000",
    "5707025B7E00F9438E7E4C0104040000F4018808A3D103000000000000000000",
    "930802587E00FE438E824C0103040000F4018808A3D103000000000000000000",
    "7B0902587E00FC438E514C0105040000F2018809A3CE03000000000000000000",
    "060A02557E00F7438E454C0104050000F101880AA3D103000000000000000000",
    "650B02557E00F4438E1C4C0105050000EF01880AA3CD03000000000000000000",
    "F50C02527E00F6438E604C0104040000F3018809A3D003000000000000000000",
    "C50D02527E00FB438E3C4C0104050000F101880AA3CC03000000000000000000",
  ],
  0xB6: [
    "7C0E2C142C1488138813000060000000",
    "7C0F2B142B1488138813000060000000",
    "B2002B142B1488138813000060000000",
    "60012B142B1488138813000060000000",
    "84022B142B1488138813000060000000",
    "FB032B142B1488138813000060000000",
    "11042C142C1488138813000060000000",
    "E2052D142D1488138813000060000000",
    "E7062D142D1488138813000060000000",
    "C1072D142D1488138813000060000000",
    "58082D142D1488138813000060000000",
    "09092C142C1488138813000060000000",
    "500A2B142B1488138813000060000000",
    "1E0B2B142B1488138813000060000000",
    "270C2A142A1488138813000060000000",
    "490D2B142B1488138813000060000000",
    "940E2B142B1488138813000060000000",
    "940F2C142C1488138813000060000000",
    "5A002C142C1488138813000060000000",
    "88012C142C1488138813000060000000",
  ],
  0xB7: [
    "450B04401004B42C000000800000000055E2048D5E2013000000000000000000",
    "070C04401004B42C000000800000000055E204855E2A13000000000000000000",
    "230D04401004B42C000000800000000055E204855E2A13000000000000000000",
    "A30E04401004B42C000000800000000055E204955E2C13000000000000000000",
    "370F04401004B42C000000800000000055E204955E2813000000000000000000",
    "460004401004B42C000000800000000055E204915E2613000000000000000000",
    "A00104401004B42C000000800000000055E204895E2613000000000000000000",
    "BC0204401004B42C000000800000000055E204855E1C13000000000000000000",
    "5A0304401004B42C000000800000000055E204795E2C13000000000000000000",
    "E50404401004B42C000000800000000055E2047D5E2E13000000000000000000",
    "240504401004B42C000000800000000055E204895E2013000000000000000000",
    "130604401004B42C000000800000000055E204855E2413000000000000000000",
    "2B0704401004B42C000000800000000055E204715E2813000000000000000000",
    "A60804401004B42C000000800000000055E204715E3013000000000000000000",
    "E30904401004B42C000000800000000055E2047D5E2213000000000000000000",
    "890A04401004B42C000000800000000055E204895E2413000000000000000000",
    "810B04401004B42C000000800000000055E204815E2E13000000000000000000",
    "8F0C04401004B42C000000800000000055E2047D5E2E13000000000000000000",
    "7F0D04401004B42C000000800000000055E204855E3213000000000000000000",
    "260E04401004B42C000000800000000055E204895E2613000000000000000000",
  ],
}


class TestLeapmotorSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (ADAS_LKA,)}
  FWD_BLACKLISTED_ADDRS = {2: [ADAS_LKA]}
  TX_MSGS = [[ADAS_LKA, 0], [POWERTRAIN_STATE, 2]]

  MAIN_BUS = 0

  STEER_ANGLE_MAX = 500
  # the 16-bit request field only spans -661..649.8 deg, so test within it
  STEER_ANGLE_TEST_MAX = 645
  DEG_TO_CAN = 50

  LATERAL_FREQUENCY = 50  # Hz

  cnt_angle_cmd = 0

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    self.VM = VehicleModel(get_safety_CP())
    self.packer = CANPackerSafety("leapmotor_b10")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.leapmotor, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    values = {"STEER_ANGLE_REQ": angle, "LCC_STEERING_ACTIVE_COPY": enabled}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("ADAS_LKA", self.MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    return self.packer.make_can_msg_safety("EPS_STEERING", self.MAIN_BUS, {"STEER_ANGLE": angle})

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("POWERTRAIN_STATE", self.MAIN_BUS, {"ASSIST_ENGAGED": enable})

  def _speed_msg(self, speed):
    kph = speed * 3.6
    values = {"WHEEL_SPEED_FL": kph, "WHEEL_SPEED_FR": kph, "WHEEL_SPEED_RL": kph, "WHEEL_SPEED_RR": kph}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("VEHICLE_STATE", self.MAIN_BUS, {"BRAKE_PRESSED": brake})

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("VEHICLE_STATE", self.MAIN_BUS, {"ACCEL_PEDAL": gas * 100})

  def test_angle_cmd_when_enabled(self):
    # lateral acceleration and jerk are tested below
    pass

  @staticmethod
  def _can_speed(speed):
    # match WHEEL_SPEEDS rounding (0.01 km/h)
    return round_speed(away_round(speed * 3.6 / 0.01) * 0.01 / 3.6)

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = self._can_speed(max(speed, 1))
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        # at the limit: safety truncates the limit to CAN units and adds 1 unit of tolerance
        limit_raw = int(get_max_angle_vm(speed, self.VM, CarControllerParams) * self.DEG_TO_CAN) + 1
        max_angle = np.clip(sign * limit_raw / self.DEG_TO_CAN, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # one unit above the limit, unless the limit is above the EPS maximum (low speed)
        max_angle_raw = sign * (limit_raw + 1) / self.DEG_TO_CAN
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = self._can_speed(max(speed, 1))
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # within limits: up, hold, down
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams) * sign)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # too fast: up, then down
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams) * sign, 2 * sign)
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  def test_cancel_only(self):
    # the camera copy of POWERTRAIN_STATE may only carry the stalk-up cancel, and only while stock ACC is engaged
    for engaged in (False, True):
      self.safety.set_cruise_engaged_prev(engaged)
      for stalk in range(4):
        msg = self.packer.make_can_msg_safety("POWERTRAIN_STATE", 2, {"STALK_PUSH": stalk})
        self.assertEqual(engaged and stalk == 3, self._tx(msg), (engaged, stalk))

  def test_rx_hook(self):
    # a valid frame passes; a bad checksum or a skipped counter is rejected
    for _ in range(10):
      self.assertTrue(self._rx(self._speed_msg(0)))
    msg = self._speed_msg(0)
    msg[0].data[0] ^= 0xFF
    self.assertFalse(self._rx(msg))

  def test_angle_sign_matches_dbc(self):
    # a real frame at right lock must give a negative measured angle in C, as in the DBC (positive = left)
    right_lock = "0B0602CAE4009EC5869A5E012C2C0000D5128658A3E803000000000000000000"  # raw 58570, about -521.6 deg
    self.assertTrue(self._rx(libsafety_py.make_CANPacket(0x18D, self.MAIN_BUS, bytes.fromhex(right_lock))))
    # one frame into an empty sample history: the new value is the minimum, the old zeros are the maximum
    self.assertEqual(self.safety.get_angle_meas_min(), 32490 - 58570)

  def test_real_frames(self):
    # unmodified frames from a real car must pass the C checksum and counter checks
    for addr, frames in REAL_FRAMES.items():
      for hexdata in frames:
        self.assertTrue(self._rx(libsafety_py.make_CANPacket(addr, self.MAIN_BUS, bytes.fromhex(hexdata))), hex(addr))
      bad = bytearray.fromhex(frames[-1])
      bad[0] ^= 0x01
      self.assertFalse(self._rx(libsafety_py.make_CANPacket(addr, self.MAIN_BUS, bytes(bad))), hex(addr))


if __name__ == "__main__":
  unittest.main()
