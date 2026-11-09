import unittest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC
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
STEERING_BYTES = {12, 13, 14, 17}  # LCC_STEERING_ACTIVE_COPY (bit 99), STEER_ANGLE_REQ (104|16), LCC_STATE (140|4)

# Real POWERTRAIN_STATE (0xB7) frames while driving with ACC engaged
REAL_POWERTRAIN_STATE = [
  "450B04401004B42C000000800000000055E2048D5E2013000000000000000000",
  "070C04401004B42C000000800000000055E204855E2A13000000000000000000",
]

# Real frames from the filmed figure-eights at the moments of hardest right and left turn, as measured by the
# comma's own gyro (livePose.angularVelocityDevice.z: +36.6 deg/s right, -44.6 deg/s left)
HARD_RIGHT = {0x18D: "0B0602CAE4009EC5869A5E012C2C0000D5128658A3E803000000000000000000", 0x233: "4C06C88947811100"}
HARD_LEFT = {0x18D: "130E02BA1D0F644386C24D010001000004128702A3E803000000000000000000", 0x233: "7E0A43A647AD3100"}


class TestLeapmotorCan(unittest.TestCase):
  def setUp(self):
    self.msg = DBC("leapmotor_b10").addr_to_msg[0x83]

  def unpack(self, dat: bytes) -> dict:
    return {n: s.get_raw_value(dat) * s.factor + s.offset for n, s in self.msg.sigs.items()}

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
            addr, out, bus = leapmotorcan.create_steering_control(packer, camera, angle, lat_active)
            self.assertEqual((addr, bus), (0x83, 0))
            self.assertEqual(out[0], leapmotor_checksum(0x83, None, bytearray(out)))
            for i in range(2, len(dat)):
              if i not in STEERING_BYTES:
                self.assertEqual(out[i], dat[i], f"byte {i}")
            got = self.unpack(bytes(out))
            self.assertAlmostEqual(got["STEER_ANGLE_REQ"], angle, delta=0.011)
            self.assertEqual(got["LCC_STEERING_ACTIVE_COPY"], int(lat_active))
            expected_state = camera["LCC_STATE"]
            if lat_active and camera["LCC_STATE"] not in leapmotorcan.LCC_STATES_STEERING:
              expected_state = leapmotorcan.LCC_STATE_ACTIVE
            self.assertEqual(got["LCC_STATE"], expected_state)
            # the ACC fields pass through untouched
            for sig in ("ACC_STATE", "ACC_ACCEL_REQ", "ACC_SET_SPEED", "ASSIST_OFF", "CONSTANT_72", "CONSTANT_152"):
              self.assertEqual(got[sig], camera[sig], sig)


if __name__ == "__main__":
  unittest.main()
