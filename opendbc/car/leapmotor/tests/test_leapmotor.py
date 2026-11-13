import unittest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC
from opendbc.car import Bus
from opendbc.car.leapmotor import leapmotorcan
from opendbc.car.leapmotor.leapmotorcan import leapmotor_checksum

# Real ADAS_LKA (0x83) frames from a 2025 global-market B10: stock LCC steering, ACC only, and assist off
REAL_ADAS_LKA = {
  "lcc": [
    "6D0C004666007D00007D007009E87E00375D007D000000000000000000000000",
    "600D004664007D00007D007008A67F004D53007D000000000000000000000000",
    "A808004664808900007D007008E87E004158007D000000000000000000000000",
  ],
  "acc": [
    "B90A005A64005F00007D007000A47F000178007D000008000000000000000000",
    "B40200465F802500007D007000087F000178007D000000000000000000000000",
    "BA03004A64C07100007D0070004F7E000178007D000008000000000000000000",
  ],
  "off": [
    "260500446411E100007D00700043E4000178007D000000000000000000000000",
    "4B0700446411E100007D007000A05A000178007D000000000000000000000000",
    "4008004464918900007D0070002D7E000178007D000000000000000000000000",
  ],
}
STEERING_BYTES = {12, 13, 14, 16, 17}  # LCC_STEERING_ACTIVE_COPY (bit 99), STEER_ANGLE_REQ (104|16), LCC_RAMP (128|16)

# Real POWERTRAIN_STATE (0xB7) frames while driving with ACC engaged
REAL_POWERTRAIN_STATE = [
  "450B04401004B42C000000800000000055E2048D5E2013000000000000000000",
  "070C04401004B42C000000800000000055E204855E2A13000000000000000000",
]

# Real frames from the filmed figure-eights at the moments of hardest right and left turn, as measured by the
# comma's own gyro (livePose.angularVelocityDevice.z: +36.6 deg/s right, -44.6 deg/s left)
HARD_RIGHT = {0x18D: "0B0602CAE4009EC5869A5E012C2C0000D5128658A3E803000000000000000000", 0x233: "4C06C88947811100"}
HARD_LEFT = {0x18D: "130E02BA1D0F644386C24D010001000004128702A3E803000000000000000000", 0x233: "7E0A43A647AD3100"}

# Real ROAD_SIGNS (0x142) frames from the camera just before and during a forward collision warning the driver reported
FCW_FRAMES = {False: "F204F4F14002001C8032120000000000", True: "050E310D4402001C8032120000000000"}


class TestLeapmotorCan(unittest.TestCase):
  def setUp(self):
    self.msg = DBC("leapmotor_b10").addr_to_msg[0x83]

  def unpack(self, dat: bytes) -> dict:
    return {n: s.get_raw_value(dat) * s.factor + s.offset for n, s in self.msg.sigs.items()}

  def test_fcw(self):
    sig = DBC("leapmotor_b10").addr_to_msg[0x142].sigs["FCW"]
    for fcw, hexdata in FCW_FRAMES.items():
      dat = bytes.fromhex(hexdata)
      self.assertEqual(sig.get_raw_value(dat), int(fcw))
      self.assertEqual(dat[0], leapmotor_checksum(0x142, None, bytearray(dat)))

  def test_sign_convention(self):
    # openpilot expects positive = left for steering angle and yaw rate
    dbc = DBC("leapmotor_b10")

    def decode(addr, hexdata, name):
      sig = dbc.addr_to_msg[addr].sigs[name]
      return sig.get_raw_value(bytes.fromhex(hexdata)) * sig.factor + sig.offset
    self.assertLess(decode(0x18D, HARD_RIGHT[0x18D], "STEER_ANGLE"), -400)
    self.assertLess(decode(0x233, HARD_RIGHT[0x233], "YAW_RATE"), -30)
    self.assertGreater(decode(0x18D, HARD_LEFT[0x18D], "STEER_ANGLE"), 400)
    self.assertGreater(decode(0x233, HARD_LEFT[0x233], "YAW_RATE"), 30)

  def test_dbc_covers_every_used_bit(self):
    # unpacking then packing a real frame (with its own counter) must reproduce it exactly
    packer = CANPacker("leapmotor_b10")
    for frames in REAL_ADAS_LKA.values():
      for hexdata in frames:
        dat = bytes.fromhex(hexdata)
        values = {k: v for k, v in self.unpack(dat).items() if k != "CHECKSUM"}
        self.assertEqual(bytes(packer.pack(0x83, values)), dat)

  def test_cancel_changes_only_stalk_and_counter(self):
    msg = DBC("leapmotor_b10").addr_to_msg[0xB7]
    for hexdata in REAL_POWERTRAIN_STATE:
      dat = bytes.fromhex(hexdata)
      vals = {n: s.get_raw_value(dat) * s.factor + s.offset for n, s in msg.sigs.items()}
      addr, out, bus = leapmotorcan.create_cancel(CANPacker("leapmotor_b10"), vals)
      self.assertEqual((addr, bus), (0xB7, 2))
      self.assertEqual(out[0], leapmotor_checksum(0xB7, None, bytearray(out)))
      self.assertEqual(out[1] & 0xF, (dat[1] + 1) & 0xF)
      self.assertEqual(out[5] & 0x3, 3)
      for i in range(2, len(dat)):
        if i != 5:
          self.assertEqual(out[i], dat[i], f"byte {i}")
      self.assertEqual(out[5] & ~0x3, dat[5] & ~0x3)

  def test_rebuild_changes_only_steering(self):
    for kind, frames in REAL_ADAS_LKA.items():
      for hexdata in frames:
        dat = bytes.fromhex(hexdata)
        camera = self.unpack(dat)
        for lat_active, angle in ((False, camera["STEER_ANGLE_REQ"]), (True, -123.46), (True, 456.78)):
          with self.subTest(kind=kind, lat_active=lat_active, angle=angle):
            packer = CANPacker("leapmotor_b10")
            ramp = 0x6B0F  # a value in the middle of the stock ramp
            addr, out, bus = leapmotorcan.create_steering_control(packer, camera, angle, lat_active, ramp)
            self.assertEqual((addr, bus), (0x83, 0))
            self.assertEqual(out[0], leapmotor_checksum(0x83, None, bytearray(out)))
            for i in range(2, len(dat)):
              if i not in STEERING_BYTES:
                self.assertEqual(out[i], dat[i], f"byte {i}")
            got = self.unpack(bytes(out))
            self.assertAlmostEqual(got["STEER_ANGLE_REQ"], angle, delta=0.011)
            self.assertEqual(got["LCC_STEERING_ACTIVE_COPY"], int(lat_active))
            # LCC_RAMP: openpilot's ramp while steering, the camera's otherwise
            self.assertEqual(got["LCC_RAMP"], ramp if lat_active else camera["LCC_RAMP"])
            # the ACC fields pass through untouched
            for sig in ("ACC_STATE", "ACC_ACCEL_REQ", "ACC_SET_SPEED", "ASSIST_OFF", "CONSTANT_72", "CONSTANT_152"):
              self.assertEqual(got[sig], camera[sig], sig)

  def test_lcc_ramp_matches_stock(self):
    # stock LCC engagement: LCC_RAMP goes 0x7801, 0x7703, 0x7605, ... one 254 step per frame, then jumps back
    straight_90 = leapmotorcan.lcc_ramp_target(0.15, 90.0)  # a typical straight road: stock median there is 31 steps
    self.assertEqual(straight_90, 0x593F)
    word, seen = leapmotorcan.LCC_RAMP_IDLE, []
    for _ in range(40):
      word = leapmotorcan.lcc_ramp_next(word, True, straight_90)
      seen.append(word)
    # two ladder steps per frame up to 25 steps (stock: one), then one per frame as stock
    self.assertEqual(seen[:3], [0x7605, 0x7409, 0x720D])
    steps_seen = [(leapmotorcan.LCC_RAMP_IDLE - w) // leapmotorcan.LCC_RAMP_STEP for w in seen]
    self.assertTrue(all((leapmotorcan.LCC_RAMP_IDLE - w) % leapmotorcan.LCC_RAMP_STEP == 0 for w in seen))  # on the ladder
    self.assertEqual(steps_seen.index(26), 12)  # 25 steps passed in 13 frames, about 0.26 s
    self.assertTrue(all(b - a == 1 for a, b in zip(steps_seen[12:], steps_seen[13:], strict=False) if b != a))
    self.assertEqual(seen.index(straight_90), 17)  # 18 frames instead of stock's 31
    self.assertEqual(seen[-1], straight_90)
    # never past the target, even when a double step would overshoot it
    shallow = leapmotorcan.LCC_RAMP_IDLE - 24 * leapmotorcan.LCC_RAMP_STEP
    target_25 = leapmotorcan.LCC_RAMP_IDLE - 25 * leapmotorcan.LCC_RAMP_STEP
    self.assertEqual(leapmotorcan.lcc_ramp_next(shallow, True, target_25), target_25)
    self.assertEqual(leapmotorcan.lcc_ramp_next(seen[-1], False, straight_90), leapmotorcan.LCC_RAMP_IDLE)
    # one step per frame toward a new target, both ways
    deeper = leapmotorcan.lcc_ramp_target(1.5, 75.0)
    self.assertEqual(leapmotorcan.lcc_ramp_next(straight_90, True, deeper), straight_90 - leapmotorcan.LCC_RAMP_STEP)
    self.assertEqual(leapmotorcan.lcc_ramp_next(deeper, True, straight_90), deeper + leapmotorcan.LCC_RAMP_STEP)
    # stock medians per lateral-acceleration band (about 90 km/h), within 2 steps, and the stock range 25-46 steps
    for alat, stock in ((0.15, 0x593F), (0.45, 0x5547), (0.8, 0x534B), (1.25, 0x4F53)):
      self.assertLessEqual(abs(leapmotorcan.lcc_ramp_target(alat, 90.0) - stock), 2 * leapmotorcan.LCC_RAMP_STEP, alat)
    for alat, v in ((0.0, 0.0), (5.0, 150.0)):
      steps = (leapmotorcan.LCC_RAMP_IDLE - leapmotorcan.lcc_ramp_target(alat, v)) // leapmotorcan.LCC_RAMP_STEP
      self.assertTrue(leapmotorcan.LCC_RAMP_STEPS_MIN <= steps <= leapmotorcan.LCC_RAMP_STEPS_MAX)
    # the real stock frames agree: idle when not steering, on the ladder within the stock range while steering
    for hexdata in REAL_ADAS_LKA["acc"] + REAL_ADAS_LKA["off"]:
      self.assertEqual(self.unpack(bytes.fromhex(hexdata))["LCC_RAMP"], leapmotorcan.LCC_RAMP_IDLE)
    for hexdata in REAL_ADAS_LKA["lcc"]:
      # stock occasionally sits 2 off the ladder (steps of -2/+2 also appear between frames)
      steps = round((leapmotorcan.LCC_RAMP_IDLE - int(self.unpack(bytes.fromhex(hexdata))["LCC_RAMP"])) / leapmotorcan.LCC_RAMP_STEP)
      self.assertLessEqual(abs(leapmotorcan.LCC_RAMP_IDLE - steps * leapmotorcan.LCC_RAMP_STEP - int(self.unpack(bytes.fromhex(hexdata))["LCC_RAMP"])), 2)
      self.assertTrue(leapmotorcan.LCC_RAMP_STEPS_MIN <= steps <= leapmotorcan.LCC_RAMP_STEPS_MAX)

  def test_ramp_boost(self):
    deepest = leapmotorcan.LCC_RAMP_IDLE - leapmotorcan.LCC_RAMP_STEP * leapmotorcan.LCC_RAMP_STEPS_MAX
    # extra steps go deeper, never beyond the deepest stock value
    base = leapmotorcan.lcc_ramp_target(0.1, 15.0)
    self.assertEqual(leapmotorcan.lcc_ramp_target(0.1, 15.0, 3), base - 3 * leapmotorcan.LCC_RAMP_STEP)
    self.assertEqual(leapmotorcan.lcc_ramp_target(0.1, 15.0, 100), deepest)
    self.assertEqual(leapmotorcan.lcc_ramp_target(3.0, 120.0, 5), deepest)

    from types import SimpleNamespace
    from opendbc.car.leapmotor.carcontroller import CarController, get_safety_CP
    from opendbc.car.leapmotor.values import CarControllerParams, DBC as LEAPMOTOR_DBC
    controller = CarController(LEAPMOTOR_DBC["LEAPMOTOR_B10"], get_safety_CP())

    def cs(angle, kph):
      return SimpleNamespace(out=SimpleNamespace(steeringAngleDeg=angle, vEgoRaw=kph / 3.6))
    controller.apply_angle_last = 90.
    for i in range(30):  # wheel 10 deg behind at 15 km/h: one step deeper per frame, up to the stock maximum
      controller.ramp_boost = controller.update_ramp_boost(True, cs(80., 15.))
      self.assertEqual(controller.ramp_boost, min(i + 1, leapmotorcan.LCC_RAMP_STEPS_MAX - leapmotorcan.LCC_RAMP_STEPS_MIN))
    controller.ramp_boost = controller.update_ramp_boost(True, cs(88., 15.))  # 2 deg behind: hold
    self.assertEqual(controller.ramp_boost, 21)
    controller.ramp_boost = controller.update_ramp_boost(True, cs(89.5, 15.))  # caught up: one step back
    self.assertEqual(controller.ramp_boost, 20)
    self.assertEqual(controller.update_ramp_boost(False, cs(80., 15.)), 0)  # not steering
    self.assertEqual(controller.update_ramp_boost(True, cs(80., CarControllerParams.RAMP_BOOST_MAX_SPEED + 1)), 0)  # fast

  def test_driver_torque_lags_the_eps(self):
    from opendbc.car.leapmotor.carstate import CarState
    from opendbc.car.leapmotor.interface import CarInterface

    def pressed(driver, eps):
      CP = CarInterface.get_non_essential_params("LEAPMOTOR_B10")
      CS = CarState(CP)
      parsers = CS.get_can_parsers(CP)
      packer = CANPacker("leapmotor_b10")
      out = []
      for i, (d, e) in enumerate(zip(driver, eps, strict=True)):
        dat = packer.make_can_msg("EPS_STEERING", 0, {"STEER_TORQUE_DRIVER": d, "STEER_TORQUE_EPS": e, "COUNTER": i % 16})[1]
        parsers[Bus.pt].update([(int((i + 1) * 1e7), [(0x18D, dat, 0)])])
        parsers[Bus.cam].update([(int((i + 1) * 1e7), [])])
        CS.update(parsers)
        out.append(CS.driver_override)
      return out

    # hands off in a roundabout at 28 km/h (route 264): the EPS swings -99 -> +98 within 0.1 s while the reading barely
    # moves; subtracting the EPS's share without its lag read above 3 for 8 of these frames and let go of the wheel
    driver = [2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0]
    eps = [9, 1, -16, -14, -22, -28, -31, -38, -44, -37, -18, -1, 15, 25, 27, 20, 17, 13, 8, -9, -37, -73, -99, -84, -52,
           -4, 58, 78, 98, 96, 75, 53, 35, 16, 0, -15]
    self.assertFalse(any(pressed(driver, eps)))
    # the driver's lane-change nudge at 126 km/h (route 25c): the EPS holds -57 against the hands, the reading stays at 0
    driver = [1, 1, 1, 1, 1, 0, 0, 0, 0, 1] + [0] * 20
    eps = [-12, -7, -7, -6, -8, -14, -28, -47, -57, -56] + [-57] * 20
    self.assertTrue(any(pressed(driver, eps)))


if __name__ == "__main__":
  unittest.main()
