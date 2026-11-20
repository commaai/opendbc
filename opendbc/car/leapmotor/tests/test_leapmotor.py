import unittest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC
from opendbc.car import Bus, structs
from opendbc.car.leapmotor import leapmotorcan
from opendbc.car.leapmotor.carcontroller import CarController, get_safety_CP
from opendbc.car.leapmotor.leapmotorcan import leapmotor_checksum
from opendbc.car.leapmotor.values import DBC as LEAPMOTOR_DBC, CarControllerParams

DBC_NAMES = LEAPMOTOR_DBC["LEAPMOTOR_B10"]

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

    def cs(angle, kph, eps=0):
      return SimpleNamespace(out=SimpleNamespace(steeringAngleDeg=angle, vEgoRaw=kph / 3.6, steeringTorqueEps=eps))
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

    # near the EPS torque at which it flags an override by itself: no deeper, then back off, even while lagging
    hold, back = CarControllerParams.RAMP_BOOST_EPS_TORQUE_HOLD, CarControllerParams.RAMP_BOOST_EPS_TORQUE_BACK
    controller.ramp_boost = 10
    for eps in (hold, -hold, back - 1, -(back - 1)):
      self.assertEqual(controller.update_ramp_boost(True, cs(80., 15., eps)), 10)
    for eps in (back, -back, -218):
      controller.ramp_boost = 10
      self.assertEqual(controller.update_ramp_boost(True, cs(80., 15., eps)), 9)
    self.assertEqual(controller.update_ramp_boost(True, cs(80., 15., hold - 1)), 11)  # below the hold: deeper again

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

  def test_take_back(self):
    from types import SimpleNamespace
    from opendbc.car import structs as car_structs
    from opendbc.car.leapmotor.carcontroller import CarController as Controller, get_safety_CP as safety_CP
    from opendbc.car.leapmotor.values import DBC as CAR_DBC

    def steering_after_let_go(wanted, wheel, kph=25.):
      controller = Controller(CAR_DBC["LEAPMOTOR_B10"], safety_CP())
      CC = car_structs.CarControl()
      CC.latActive = True
      CC.actuators.steeringAngleDeg = wanted
      CS = SimpleNamespace(driver_override=False, adas_lka=None, powertrain_state=None,
                           out=SimpleNamespace(steeringAngleDeg=wheel, vEgoRaw=kph / 3.6, steeringTorqueEps=0))
      controller.update(CC.as_reader(), CS, 0)
      return controller.lat_active_last

    # right after the driver lets go in a bend the wheel is 11-27 deg from the model's angle: take it back at once
    self.assertTrue(steering_after_let_go(100., 73.))
    self.assertTrue(steering_after_let_go(100., 100. - 44.))
    # the case the check was made for: wheel at +308 deg, model at +24, at walking pace
    self.assertFalse(steering_after_let_go(24., 308., kph=5.))

  def test_eps_status_bit_is_not_torque(self):
    # Real EPS_STEERING frames from route 26e (22:19:39, openpilot steering at 60 km/h): bit 180 turns on for 1 s. Read
    # as part of the torque it added 4096, which looked like a driver torque of +264 and let go of the wheel mid-turn.
    frames = ["350B0265810000448EF25001070700002B02870FA30904000000000000000000",
              "5E0C0265810103448EF75001070700002B02870FA30814000000000000000000"]
    sigs = DBC("leapmotor_b10").addr_to_msg[0x18D].sigs
    for hexdata, (torque, status) in zip(frames, ((33, 0), (32, 1)), strict=True):
      dat = bytes.fromhex(hexdata)
      self.assertEqual(dat[0], leapmotor_checksum(0x18D, None, bytearray(dat)))
      eps = sigs["STEER_TORQUE_EPS"]
      self.assertEqual(eps.get_raw_value(dat) * eps.factor + eps.offset, torque)
      self.assertEqual(sigs["EPS_STATUS_180"].get_raw_value(dat), status)

    from opendbc.car.leapmotor.carstate import CarState
    from opendbc.car.leapmotor.interface import CarInterface
    CP = CarInterface.get_non_essential_params("LEAPMOTOR_B10")
    CS = CarState(CP)
    parsers = CS.get_can_parsers(CP)
    for i in range(20):
      parsers[Bus.pt].update([(int((i + 1) * 1e7), [(0x18D, bytes.fromhex(frames[min(i, 1)]), 0)])])
      parsers[Bus.cam].update([(int((i + 1) * 1e7), [])])
      ret = CS.update(parsers)
      self.assertLess(abs(ret.steeringTorqueEps), 100)
      self.assertFalse(CS.driver_override)

  def test_accel_replaces_only_the_request(self):
    # with openpilot longitudinal, only ACC_ACCEL_REQ changes besides the steering fields
    for hexdata in REAL_ADAS_LKA["acc"]:
      dat = bytes.fromhex(hexdata)
      camera = self.unpack(dat)
      for accel in (-3.5, -0.75, 0.0, 1.2):
        _, out, _ = leapmotorcan.create_steering_control(CANPacker("leapmotor_b10"), camera, camera["STEER_ANGLE_REQ"], False,
                                                         accel=accel)
        got = self.unpack(bytes(out))
        self.assertAlmostEqual(got["ACC_ACCEL_REQ"], accel, delta=0.026)
        for i in range(2, len(dat)):
          if i not in STEERING_BYTES | {4}:
            self.assertEqual(out[i], dat[i], f"byte {i}")

  def test_long_accel(self):
    CP = get_safety_CP()
    CP.openpilotLongitudinalControl = True
    CC = structs.CarControl()
    controller = CarController(DBC_NAMES, CP)
    limits = CarControllerParams
    cases = [
      # longActive, ACC_STATE, camera request, openpilot request -> sent (None = the camera's)
      (False, 3, -1.0, -0.5, None),  # openpilot not in control
      (True, 2, 0.0, -0.5, None),    # stock ACC not engaged
      (True, 1, 1.5, -0.5, None),    # accelerator override
      (True, 3, -1.0, -0.5, -0.5),   # openpilot may brake less than stock ACC's normal braking
      (True, 3, -1.0, 5.0, limits.ACCEL_MAX),
      (True, 3, -1.0, -9.0, limits.ACCEL_MIN),
      (True, 3, -2.0, -0.5, -2.0),   # but never less than the camera's hard braking
      (True, 3, -2.6, -3.0, -3.0),
    ]
    for long_active, acc_state, camera_accel, accel, expected in cases:
      with self.subTest(long_active=long_active, acc_state=acc_state, camera_accel=camera_accel, accel=accel):
        CC.longActive = long_active
        CC.actuators.accel = accel
        self.assertEqual(controller.long_accel(CC, {"ACC_STATE": acc_state, "ACC_ACCEL_REQ": camera_accel}), expected)
    # without openpilot longitudinal the camera's request always goes through
    CP.openpilotLongitudinalControl = False
    CC.longActive = True
    self.assertIsNone(CarController(DBC_NAMES, CP).long_accel(CC, {"ACC_STATE": 3, "ACC_ACCEL_REQ": -1.0}))

  def test_override_while_the_wheel_follows(self):
    # route 0000000a 12:31:42.9, left bend at 33 km/h, the driver's hand resting on the wheel: the EPS holds about 130 into
    # the turn, the reading follows less of it than assumed and the EPS sets its own flag. The previous rule let go and
    # the wheel sprang back 14 deg; the tracking rule keeps steering. A driver turning against the EPS still counts.
    from opendbc.car.leapmotor.carstate import CarState
    from opendbc.car.leapmotor.interface import CarInterface

    def run(frames, angle=-41.):
      CP = CarInterface.get_non_essential_params("LEAPMOTOR_B10")
      CS = CarState(CP)
      parsers = CS.get_can_parsers(CP)
      packer = CANPacker("leapmotor_b10")
      for i, (driver, eps, flag) in enumerate(frames):
        dat = packer.make_can_msg("EPS_STEERING", 0, {"STEER_TORQUE_DRIVER": driver, "STEER_TORQUE_EPS": eps, "DRIVER_OVERRIDE": flag,
                                                      "STEER_ANGLE": angle, "COUNTER": i % 16})[1]
        parsers[Bus.pt].update([(int((i + 1) * 1e7), [(0x18D, dat, 0)])])
        parsers[Bus.cam].update([(int((i + 1) * 1e7), [])])
        CS.update(parsers)
      return CS

    hands_off = [(-4, 120, 0)] * 40 + [(-5, 133, 0)] * 3 + [(-3, 133, 1)] * 3
    CS = run(hands_off)
    self.assertTrue(CS.driver_override)
    self.assertFalse(CS.driver_override_tracking)
    # the driver straightens the wheel against the EPS: the reading goes well past the EPS's share
    CS = run([(-4, 120, 0)] * 40 + [(-16, 133, 1)] * 8)
    self.assertTrue(CS.driver_override)
    self.assertTrue(CS.driver_override_tracking)

  def test_tracking_rule_only_while_the_wheel_follows(self):
    from types import SimpleNamespace
    from opendbc.car import structs as car_structs
    from opendbc.car.leapmotor.carcontroller import CarController as Controller, get_safety_CP as safety_CP
    from opendbc.car.leapmotor.values import DBC as CAR_DBC

    def steering(wheel, command=-43.):
      controller = Controller(CAR_DBC["LEAPMOTOR_B10"], safety_CP())
      CC = car_structs.CarControl()
      CC.latActive = True
      CC.actuators.steeringAngleDeg = command
      CS = SimpleNamespace(driver_override=False, driver_override_tracking=False, adas_lka=None,
                           powertrain_state=None, out=SimpleNamespace(steeringAngleDeg=-43., vEgoRaw=33 / 3.6, steeringTorqueEps=130))
      controller.lat_active_last, controller.apply_angle_last = True, command  # steering, the wheel following
      CS.driver_override = True
      CS.out.steeringAngleDeg = wheel
      controller.update(CC.as_reader(), CS, 0)
      return controller.lat_active_last

    self.assertTrue(steering(-41.))  # follows within 3 deg: the tracking rule, which sees no driver
    self.assertTrue(steering(1., command=2.))  # also near straight ahead
    self.assertFalse(steering(-36.))  # held 7 deg short: the previous rule lets go
    self.assertFalse(steering(3., command=12.))  # held straight against a 12 deg command (route 00000010 23:18:35)


if __name__ == "__main__":
  unittest.main()
