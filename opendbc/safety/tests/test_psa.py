#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py

LANE_KEEP_ASSIST = 0x3F2


class TestPsaSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  DBC = "psa_aee2010_r3"
  SAFETY_MODEL = CarParams.SafetyModel.psa

  RELAY_MALFUNCTION_ADDRS = {0: (LANE_KEEP_ASSIST,)}
  FWD_BLACKLISTED_ADDRS = {2: [LANE_KEEP_ASSIST]}
  TX_MSGS = [[1010, 0]]

  MAIN_BUS = 0
  ADAS_BUS = 1
  CAM_BUS = 2

  STEER_ANGLE_MAX = 390
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2.5, 1.5, .2]
  ANGLE_RATE_DOWN = [5., 2., .3]

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"SET_ANGLE": angle, "TORQUE_FACTOR": 100 if enabled else 0}
    return self.packer.make_can_msg_safety("LANE_KEEP_ASSIST", self.MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"ANGLE": angle}
    return self.packer.make_can_msg_safety("STEERING_ALT", self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"RVV_ACC_ACTIVATION_REQ": enable}
    return self.packer.make_can_msg_safety("HS2_DAT_MDD_CMD_452", self.ADAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"VITESSE_VEHICULE_ROUES": speed * 3.6}
    return self.packer.make_can_msg_safety("HS2_DYN_ABR_38D", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"P013_MainBrake": brake}
    return self.packer.make_can_msg_safety("Dat_BSI", self.CAM_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"P002_Com_rAPP": int(gas * 100)}
    return self.packer.make_can_msg_safety("Dyn_CMM", self.MAIN_BUS, values)

  def test_rx_hook(self):
    # speed
    for _ in range(10):
      self.assertTrue(self._rx(self._speed_msg(0)))
    msg = self._speed_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))

    # cruise
    for _ in range(10):
      self.assertTrue(self._rx(self._pcm_status_msg(0)))
    msg = self._pcm_status_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))
    msg = self._pcm_status_msg(0)
    # write to unused payload byte
    msg[0].data[6] = 0xAB
    self.assertTrue(self._rx(msg))


class TestPsaStockSafety(TestPsaSafetyBase):
  pass


class TestPsaIgnition(common.SafetyTestBase):
  DBC = "psa_aee2010_r3"
  SAFETY_MODEL = None

  TX_MSGS: list = []

  def setUp(self):
    super().setUp()
    self.counter = 0
    # a bad Dyn_EasyMove resets the check, its state is static
    self.safety.ignition_can_hook(self._easy_move_msg(valid=False))

  # Dyn_EasyMove, counter and checksum in byte 7
  def _easy_move_msg(self, valid=True):
    self.counter = (self.counter + 1) % 16
    dat = bytearray(8)
    dat[7] = self.counter << 4
    dat[7] |= (-(3 + sum((b >> 4) + (b & 0xF) for b in dat)) + (0 if valid else 1)) & 0xF
    return libsafety_py.make_CANPacket(0x3AD, 0, bytes(dat))

  # Dyn2_CMM, bit 41: EV running, bit 42: ICE running
  def _msg(self, byte5):
    return libsafety_py.make_CANPacket(0x348, 0, bytes([0, 0, 0, 0, 0, byte5, 0, 0]))

  def test_ignition_on(self):
    for _ in range(3):
      self.safety.ignition_can_hook(self._msg(0x3))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._easy_move_msg())
    self.safety.ignition_can_hook(self._msg(0x3))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    for _ in range(3):
      self.safety.ignition_can_hook(self._easy_move_msg())
    self.safety.ignition_can_hook(self._msg(0x3))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x1))
    self.assertFalse(self.safety.get_ignition_can())

  def test_ignition_bad_easy_move(self):
    for _ in range(3):
      self.safety.ignition_can_hook(self._easy_move_msg())
    self.safety.ignition_can_hook(self._easy_move_msg(valid=False))
    # VW camera 0x348 seen on bus 0 while the harness relay is open
    self.safety.ignition_can_hook(self._msg(0xFF))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
    unittest.main()
