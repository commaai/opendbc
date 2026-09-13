#!/usr/bin/env python3
import unittest

from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestMazdaLongitudinalSafety(TestMazdaSafety, common.LongitudinalAccelSafetyTest):

  TX_MSGS = TestMazdaSafety.TX_MSGS + [[addr, bus] for bus in (0, 2)
                                       for addr in (0x21b, 0x21c, 0x499, *range(0x361, 0x367))] + [[0x764, 0]]
  MAX_ACCEL, MIN_ACCEL = 2.0, -3.5

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.LONG)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("PEDALS", 0, {"ACC_ACTIVE": enable})

  def _accel_msg(self, accel: float, bus: int = 0, active: bool = False, **values):
    return self.packer.make_can_msg_safety("CRZ_INFO", bus, {"ACCEL_CMD": accel, "ACC_ACTIVE": active, **values})

  def _crz_ctrl_msg(self, active: bool, bus: int = 0):
    return self.packer.make_can_msg_safety("CRZ_CTRL", bus, {"CRZ_ACTIVE": active})

  def _press_engage(self):
    self._rx(self._button_msg(resume=True))

  def test_enable_control_allowed_from_cruise(self):
    self._press_engage()
    super().test_enable_control_allowed_from_cruise()

  def test_engagement_button_guard(self):
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self._press_engage()
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._button_msg(cancel=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_active_bits_require_controls_allowed(self):
    for bus in (0, 2):
      self.safety.set_controls_allowed(False)
      self.assertFalse(self._tx(self._accel_msg(0., bus=bus, active=True)))
      self.assertFalse(self._tx(self._crz_ctrl_msg(True, bus)))

  def test_stock_crz_info_standby_allowed(self):
    def standby_frame(d4, d5, counter):
      dat = bytes([0x01, 0xff, 0xe3, 0xff, d4, d5, counter])
      return dat + bytes([(0xff - sum(dat)) & 0xff])

    for bus, d4, d5, counter in ((0, 0xc0, 0x00, 0), (2, 0xc4, 0x80, 15)):
      self.assertTrue(self._tx(make_msg(bus, 0x21b, dat=standby_frame(d4, d5, counter))))
      self.assertFalse(self._tx(make_msg(bus, 0x21b, dat=bytes.fromhex("01ffe3ffc0000000"))))

  def test_empty_radar_frames(self):
    radar_messages = ((0x499, "0008c00000000000"), (0x361, "fff7fefe1fc00080"),
                      (0x362, "fff7fefe1fc78c80"), (0x363, "fff7fefe1fc00000"), (0x364, "fff7fefe1fc00000"),
                      (0x365, "fff7fe7ffbff3fc0"), (0x366, "fff7fe7ffbff3fc0"))
    for bus in (0, 2):
      for addr, hex_data in radar_messages:
        self.assertTrue(self._tx(make_msg(bus, addr, dat=bytes.fromhex(hex_data))))
      self.assertFalse(self._tx(make_msg(bus, 0x361, dat=bytes.fromhex("fff7fefe1ec00080"))))

  def test_radar_uds_allowlist(self):
    self.assertTrue(self._tx(make_msg(0, 0x764, dat=bytes.fromhex("023e800000000000"))))
    self.assertTrue(self._tx(make_msg(0, 0x764, dat=bytes.fromhex("0210020000000000"))))
    self.assertTrue(self._tx(make_msg(0, 0x764, dat=bytes.fromhex("0210010000000000"))))
    self.assertFalse(self._tx(make_msg(0, 0x764, dat=bytes.fromhex("0322f19000000000"))))
    self.assertFalse(self._tx(make_msg(2, 0x764, dat=bytes.fromhex("023e800000000000"))))


class TestMazdaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
