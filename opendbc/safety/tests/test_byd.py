#!/usr/bin/env python3
import types
import unittest

from opendbc.car.byd.bydcan import byd_checksum, CHECKSUM_KEY
from opendbc.car.byd.values import LKAS_HUD_PASSTHROUGH
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

STEERING_MODULE_ADAS = 0x1E2
LKAS_HUD_ADAS = 0x316
ACC_CMD = 0x32E  # blocked: not in the TX allowlist
ACC_HUD_ADAS = 0x32D

MAIN_BUS = 0
CAM_BUS = 2


def sign(msg):
  """Re-pack a message with a valid BYD checksum in the last byte."""
  dat = bytes(bytearray(msg.data[0:8]))
  dat = dat[:-1] + bytes([byd_checksum(CHECKSUM_KEY, dat[:-1])])
  return libsafety_py.make_CANPacket(msg.addr, msg.bus, dat)


class TestBydSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  TX_MSGS = [[STEERING_MODULE_ADAS, MAIN_BUS], [LKAS_HUD_ADAS, MAIN_BUS]]
  RELAY_MALFUNCTION_ADDRS = {MAIN_BUS: (STEERING_MODULE_ADAS, LKAS_HUD_ADAS)}
  # nothing is statically blocked: the camera keeps the EPS and the HUD until openpilot transmits
  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  STEER_ANGLE_MAX = 90
  # sweep stays inside the EPS limit: enforce_angle_error clamps commands to max_angle
  STEER_ANGLE_TEST_MAX = 80
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2.5, 1.5, .4]
  ANGLE_RATE_DOWN = [2.5, 1.5, .6]

  def setUp(self):
    self.packer = CANPackerSafety("byd_general")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    # STEER_REQ_ACTIVE_LOW is not the inverse of STEER_REQ on this car; the camera holds it at 0
    values = {"STEER_ANGLE": angle, "STEER_REQ": enabled, "STEER_REQ_ACTIVE_LOW": 0}
    return self.packer.make_can_msg_safety("STEERING_MODULE_ADAS", MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE_2": angle}
    return self.packer.make_can_msg_safety("STEER_MODULE_2", MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    # ACC_HUD_ADAS.ACC_ON1/ON2 are only the main-switch/standby state; the engaged flag
    # panda watches is in ACC_CMD (CMD_REQ_ACTIVE_LOW = 0 while the stock ACC commands)
    values = {"ACC_ON_1": enable, "ACC_ON_2": enable, "CMD_REQ_ACTIVE_LOW": 0 if enable else 1}
    return sign(self.packer.make_can_msg_safety("ACC_CMD", CAM_BUS, values))

  def _speed_msg(self, speed):
    # carstate applies a 40/53 correction to the raw DBC value; mirror it here
    kph = speed * 3.6 * (53.0 / 40.0)
    values = {"WHEELSPEED_FL": kph, "WHEELSPEED_FR": kph, "WHEELSPEED_BL": kph}
    return self.packer.make_can_msg_safety("WHEEL_SPEED", MAIN_BUS, values)

  def _speed_msg_2(self, speed: float):
    return None

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL": 0.5 if brake else 0.0}
    return sign(self.packer.make_can_msg_safety("PEDAL", MAIN_BUS, values))

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return sign(self.packer.make_can_msg_safety("PEDAL", MAIN_BUS, values))

  def test_camera_keeps_eps_until_we_transmit(self):
    """The camera's steering command forwards normally, and is blocked only while
    openpilot is actually sending its own — so the EPS never has two masters or none."""
    self.assertEqual(0, self.safety.safety_fwd_hook(CAM_BUS, STEERING_MODULE_ADAS))

    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(10)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(CAM_BUS, STEERING_MODULE_ADAS))

    # after openpilot goes quiet the camera gets the wheel back
    self.safety.set_timer(int(0.2 * 1e6))
    self.assertEqual(0, self.safety.safety_fwd_hook(CAM_BUS, STEERING_MODULE_ADAS))

  def test_blocked_tx_does_not_steal_the_wheel(self):
    """A rejected command must not block the camera — that would leave nobody steering."""
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._angle_cmd_msg(30, True)))
    self.assertEqual(0, self.safety.safety_fwd_hook(CAM_BUS, STEERING_MODULE_ADAS))

  def test_camera_keeps_hud_until_we_transmit(self):
    """The HUD hands over on the same terms as the steering command."""
    self.assertEqual(0, self.safety.safety_fwd_hook(CAM_BUS, LKAS_HUD_ADAS))

    self.assertTrue(self._tx(self.packer.make_can_msg_safety("LKAS_HUD_ADAS", MAIN_BUS, {})))
    self.assertEqual(-1, self.safety.safety_fwd_hook(CAM_BUS, LKAS_HUD_ADAS))

    self.safety.set_timer(int(0.2 * 1e6))
    self.assertEqual(0, self.safety.safety_fwd_hook(CAM_BUS, LKAS_HUD_ADAS))

  def test_hud_tx_does_not_block_the_steering_command(self):
    """The HUD carries no safety checks, so sending one must never let openpilot hold the
    camera's steering command off the bus — that would leave nobody steering."""
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("LKAS_HUD_ADAS", MAIN_BUS, {})))
    self.assertEqual(0, self.safety.safety_fwd_hook(CAM_BUS, STEERING_MODULE_ADAS))

  def test_checksum_rejects_corrupt_frames(self):
    """A frame whose checksum doesn't match must invalidate rx and drop controls."""
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._rx(self._user_brake_msg(False)))

    bad = self.packer.make_can_msg_safety("PEDAL", MAIN_BUS, {"BRAKE_PEDAL": 0.0})
    bad.data[7] = 0xAA  # deliberately wrong checksum
    self.assertFalse(self._rx(bad))
    self.assertFalse(self.safety.get_controls_allowed())


class TestBydSafety(TestBydSafetyBase):
  def test_reproduces_real_camera_frames(self):
    """Our steering frame must be byte-identical to the camera's apart from the angle.
    These payloads were captured from the car's own CAN bus."""
    from opendbc.can.packer import CANPacker
    from opendbc.car.byd import bydcan

    pk = CANPacker("byd_general")
    for hexs, angle, cnt in (("2b 55 eb ff ff ff 0f 88", -0.1, 0),
                             ("2b 55 eb fe ff ff ff 99", -0.2, 15),
                             ("3b e4 ee c0 ff ff 3f f5", -6.4, 3),
                             ("33 c4 ee b6 ff ff 8f d7", -7.4, 8)):
      want = bytes.fromhex(hexs.replace(" ", ""))
      template = {"UNKNOWN": (want[0] << 6) | (want[1] >> 2),
                  "SET_ME_X01": want[1] & 0x3, "SET_ME_XE": want[2] & 0xF}
      _, got, _ = bydcan.create_steering_control(pk, angle, template, cnt)
      self.assertEqual(want, got, f"expected {want.hex(' ')} got {got.hex(' ')}")

  def test_default_template_is_the_camera_frame(self):
    """With a blocked lens the camera never steers and never gives us a template, so the
    default must still be a frame the car itself emits: bytes 0-2 = 2b 55 eb. Anything
    else here — in particular a value that walks — makes the car drop its whole ADAS."""
    from opendbc.can.packer import CANPacker
    from opendbc.car.byd import bydcan
    from opendbc.car.byd.carstate import STEER_TEMPLATE_DEFAULT

    pk = CANPacker("byd_general")
    _, dat, _ = bydcan.create_steering_control(pk, 0.0, STEER_TEMPLATE_DEFAULT, 0)
    self.assertEqual(bytes.fromhex("2b55eb"), dat[:3], f"got {dat.hex(' ')}")
    self.assertTrue((dat[2] >> 5) & 1)          # STEER_REQ set
    self.assertEqual(dat[7], byd_checksum(CHECKSUM_KEY, dat[:-1]))

  def test_template_is_constant_over_a_steering_episode(self):
    """The regression guard: bytes 0-2 must be identical on every frame we send, however
    long we steer for. The car tolerates a walking SET_ME_XE/SET_ME_X01 for tens of
    seconds and then shuts the stock ADAS down for the rest of the drive."""
    from opendbc.car.byd.carcontroller import CarController
    from opendbc.car.byd.values import CarControllerParams
    from opendbc.car import Bus

    CP = CarParams(carFingerprint="BYD_ATTO3", openpilotLongitudinalControl=False)
    cc = CarController({Bus.pt: "byd_general"}, CP)

    ns = types.SimpleNamespace
    actuators = ns(steeringAngleDeg=0.0, as_builder=lambda: ns(steeringAngleDeg=0.0))
    CC = ns(actuators=actuators, latActive=True, enabled=True,
            hudControl=ns(setSpeed=30.0, leadVisible=False), cruiseControl=ns(cancel=False))
    CS = ns(steer_template=None, lkas_hud=dict.fromkeys(LKAS_HUD_PASSTHROUGH, 0),
            out=ns(vEgoRaw=25.0, steeringAngleDeg=0.0, steeringTorque=0.0, standstill=False,
                   cruiseState=ns(speed=30.0)))

    seen = set()
    for _ in range(2000 * CarControllerParams(CP).STEER_STEP):  # 2000 commands, ~40 s at 50 Hz
      _, can_sends = cc.update(CC, CS, 0)
      for addr, dat, _bus in can_sends:
        if addr == STEERING_MODULE_ADAS:
          seen.add(bytes(dat[:3]))
    self.assertEqual({bytes.fromhex("2b55eb")}, seen, f"frame varied: {sorted(seen)}")

  def test_hud_reports_lkas_active_while_steering(self):
    """The cluster must be told openpilot has the wheel, since the camera will have dropped
    its own LKAS by then. STEER_ACTIVE_ACTIVE_LOW stays 0 — it is not the inverse."""
    from opendbc.can.packer import CANPacker
    from opendbc.car.byd import bydcan

    pk = CANPacker("byd_general")
    _, dat, _ = bydcan.create_lkas_hud(pk, dict.fromkeys(LKAS_HUD_PASSTHROUGH, 0), 0)
    self.assertEqual(1, (dat[0] >> 5) & 1)   # STEER_ACTIVE_1_1   5|1@0+
    self.assertEqual(1, (dat[4] >> 5) & 1)   # STEER_ACTIVE_1_2  37|1@0+
    self.assertEqual(1, (dat[4] >> 3) & 1)   # STEER_ACTIVE_1_3  35|1@0+
    self.assertEqual(0, (dat[4] >> 4) & 1)   # STEER_ACTIVE_ACTIVE_LOW  36|1@0+
    self.assertEqual(dat[7], byd_checksum(CHECKSUM_KEY, dat[:-1]))

  def test_steer_template_latches_only_the_steady_camera_frame(self):
    """Replay of the frame sequence the camera really sends: a couple of idle and
    transitional frames, then 2b 55 eb held for the episode. Only the steady value may be
    adopted — a transitional one would be sent for the rest of the drive."""
    from opendbc.car.byd.carstate import CarState, STEER_TEMPLATE_DEFAULT

    CS = CarState.__new__(CarState)
    CS.steer_template = None
    CS._template_candidate = None
    CS._template_frames = 0

    def frame(hexs, steer_req=1):
      b = bytes.fromhex(hexs.replace(" ", ""))
      return {"STEER_REQ": steer_req, "UNKNOWN": (b[0] << 6) | (b[1] >> 2),
              "SET_ME_X01": b[1] & 0x3, "SET_ME_XE": b[2] & 0xF}

    CS.update_steer_template(frame("00 00 e0", steer_req=0))
    self.assertIsNone(CS.steer_template)

    # ramp in: idle bytes with STEER_REQ already set, then two one-off transitional frames
    for hexs in ("00 00 e0", "00 00 e0", "04 ed eb", "19 99 eb"):
      CS.update_steer_template(frame(hexs))
    self.assertIsNone(CS.steer_template, "latched a transitional frame")

    for _ in range(10):
      CS.update_steer_template(frame("2b 55 eb"))
    self.assertEqual(STEER_TEMPLATE_DEFAULT, CS.steer_template)

    # the episode ends; the latched value is kept for the next one
    CS.update_steer_template(frame("00 00 e0", steer_req=0))
    self.assertEqual(STEER_TEMPLATE_DEFAULT, CS.steer_template)

  def test_acc_cmd_not_allowed(self):
    """Longitudinal is stock-only; ACC_CMD must never be transmitted."""
    self.safety.set_controls_allowed(True)
    values = {"ACCEL_CMD": 100}
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("ACC_CMD", MAIN_BUS, values)))


if __name__ == "__main__":
  unittest.main()
