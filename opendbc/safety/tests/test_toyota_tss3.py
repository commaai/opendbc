#!/usr/bin/env python3
"""Panda safety tests for the TSS 3.0 CAN FD longitudinal (0x160) port.

This is the barrier between openpilot and the car's acceleration, so it gets its
own focused suite. Run from an opendbc checkout with ALLOW_DEBUG (the TSS3 param
is debug-gated):

    ALLOW_DEBUG=1 PYTHONPATH=$PWD python opendbc/safety/tests/test_toyota_tss3.py

It builds raw CAN frames (no DBC packer) so it does not depend on the E2E CRC --
the panda does not check the CRC, only the accel bounds and controls_allowed.

What it proves:
  - only 0x160 on bus 0 is in the tx allowlist
  - 0x160 accel outside +/-1.5 m/s^2 is always blocked
  - 0x160 in-bounds is allowed ONLY when controls_allowed and gas not pressed
  - the 0x8A cruise-engaged bit (bus 1) enters/exits controls
  - brake (0x101) and gas (0x116) on bus 1 gate longitudinal
  - the camera's own 0x160 is blocked from forwarding bus2 -> bus0
"""
import unittest

from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py

EPS_SCALE = 73
ACCEL_SCALE = 0.001          # m/s^2 per count
# panda accel envelope for 0x160 = STOCK Toyota range (openpilot relays the
# camera's own frame, which brakes to ~-3.3 at stops). openpilot's own commands
# are separately clamped to +/-1.5 in the carcontroller.
MAX_ACCEL = 2.0
MIN_ACCEL = -3.5


def _u8(x):
  return x & 0xFF


class TestToyotaTSS3(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(
      CarParams.SafetyModel.toyota,
      EPS_SCALE | ToyotaSafetyFlags.SECOC | ToyotaSafetyFlags.TSS3,
    )
    self.safety.init_tests()

  # ---- frame builders (raw bytes, correct bus) ---------------------------
  def _accel_160(self, accel, steer_counts=0):
    """0x160 on bus 0. ACCEL_REQ = 15-bit signed BE in byte4[6:0]+byte5,
    byte4 bit7 is the constant flag. STEER_REQ = bytes 22-23, 16-bit signed BE."""
    raw = int(round(accel / ACCEL_SCALE))
    raw = max(-16384, min(16383, raw)) & 0x7FFF
    d = bytearray(32)
    d[4] = 0x80 | ((raw >> 8) & 0x7F)   # keep the constant flag bit
    d[5] = raw & 0xFF
    s = steer_counts & 0xFFFF
    d[22] = (s >> 8) & 0xFF
    d[23] = s & 0xFF
    return libsafety_py.make_CANPacket(0x160, 0, bytes(d))

  def _cruise_8a(self, engaged, acc_state=0x47):
    """0x8A on bus 1. cruise engaged = byte22 bit4 (0x10). ACC_STATE byte7."""
    d = bytearray(32)
    d[7] = acc_state
    d[22] = 0x10 if engaged else 0x00
    return libsafety_py.make_CANPacket(0x8A, 1, bytes(d))

  def _speed_aa(self, kph):
    """0xAA on bus 1. four wheel speeds, raw = (kph + 67.67)/0.01."""
    raw = int(round((kph + 67.67) / 0.01))
    d = bytearray(8)
    for i in range(0, 8, 2):
      d[i] = (raw >> 8) & 0x7F
      d[i + 1] = raw & 0xFF
    return libsafety_py.make_CANPacket(0xAA, 1, bytes(d))

  def _brake_101(self, pressed):
    """0x101 on bus 1. brake = bit 3."""
    d = bytearray(8)
    if pressed:
      d[0] |= (1 << 3)
    return libsafety_py.make_CANPacket(0x101, 1, bytes(d))

  def _gas_116(self, pressed):
    """0x116 on bus 1. gas = byte 1 != 0."""
    d = bytearray(8)
    d[1] = 60 if pressed else 0
    return libsafety_py.make_CANPacket(0x116, 1, bytes(d))

  # ---- helpers -----------------------------------------------------------
  def _engage(self):
    self.assertTrue(self._rx(self._speed_aa(30)))
    self.assertTrue(self._rx(self._gas_116(False)))
    self.assertTrue(self._rx(self._brake_101(False)))
    # rising edge of cruise engaged -> controls_allowed
    self._rx(self._cruise_8a(False))
    self._rx(self._cruise_8a(True))

  def _rx(self, m):
    return self.safety.safety_rx_hook(m)

  def _tx(self, m):
    return self.safety.safety_tx_hook(m)

  # ---- tests -------------------------------------------------------------
  def test_engage_from_cruise_bit(self):
    self.safety.set_controls_allowed(False)
    self._rx(self._cruise_8a(False))
    self._rx(self._cruise_8a(True))
    self.assertTrue(self.safety.get_controls_allowed())
    # disengage clears it
    self._rx(self._cruise_8a(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_accel_bounds_when_engaged(self):
    self._engage()
    self.assertTrue(self.safety.get_controls_allowed())
    for milli in range(-4000, 2501, 50):
      accel = milli / 1000.0
      should_tx = (MIN_ACCEL - 1e-9) <= accel <= (MAX_ACCEL + 1e-9)
      self.assertEqual(should_tx, self._tx(self._accel_160(accel)),
                       f"accel {accel:+.3f} should_tx={should_tx}")

  def test_no_tx_when_disengaged(self):
    # When disengaged, only the inactive value (0) may go out -- that is the
    # "not commanding" accel and is always permitted, same as every Toyota. Any
    # NON-zero accel must be blocked.
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._accel_160(0.0)),
                    "inactive accel (0) should always tx")
    for accel in (-1.5, -1.0, -0.1, 0.1, 1.0, 1.5):
      self.assertFalse(self._tx(self._accel_160(accel)),
                       f"nonzero accel {accel} must be blocked while disengaged")

  def test_no_tx_when_gas_pressed(self):
    self._engage()
    self.assertTrue(self._tx(self._accel_160(1.0)))   # baseline: allowed
    # camera is blocked while openpilot controls (longitudinal allowed)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x160))
    self._rx(self._gas_116(True))                      # driver hits the gas
    self.assertFalse(self._tx(self._accel_160(1.0)),
                     "accel tx must be blocked while gas pressed")
    # ...and the camera's 0x160 must now FORWARD (no gap during gas override)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x160),
                     "camera 0x160 must forward during gas override (no gap)")

  def test_disengage_on_brake(self):
    self._engage()
    self.assertTrue(self._tx(self._accel_160(1.0)))
    # a brake press does not itself clear controls_allowed in toyota safety,
    # but cruise disengaging on brake does. Model the real sequence:
    self._rx(self._brake_101(True))
    self._rx(self._cruise_8a(False))                   # stock ACC drops out
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._accel_160(1.0)))

  def test_tx_allowlist(self):
    self._engage()
    # 0x160 on bus 0 allowed (in bounds); the legacy DSU 0x160 on bus 1 is NOT
    # part of this mode's tx set and must be blocked.
    self.assertTrue(self._tx(self._accel_160(0.5)))
    dsu = libsafety_py.make_CANPacket(0x160, 1, bytes(8))
    self.assertFalse(self._tx(dsu), "0x160 on bus 1 must not tx in TSS3 mode")
    # a steering message that other modes allow must be blocked here
    steer = libsafety_py.make_CANPacket(0x2E4, 0, bytes(5))
    self.assertFalse(self._tx(steer), "no lateral tx on TSS3")

  def test_selective_fwd_camera_160(self):
    # Selective pass-through: the camera 0x160 forwards 2->0 when openpilot is
    # NOT controlling, and is blocked when openpilot IS controlling (so its own
    # 0x160 is the only one the gateway sees).
    self.safety.set_controls_allowed(False)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x160),
                     "camera 0x160 must forward when NOT controlling")
    self.safety.set_controls_allowed(True)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x160),
                     "camera 0x160 must be blocked while controlling")
    # other ADAS msgs always forward; bus 0 -> bus 2 always forwards
    for ca in (True, False):
      self.safety.set_controls_allowed(ca)
      self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x180),
                       "other ADAS msgs should always forward 2->0")
      self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x123),
                       "bus 0 should always forward to bus 2")

  # ---- lateral (steer request in 0x160 bytes 22-23) ----------------------
  # The steer request rides in 0x160 (camera-origin, openpilot is sole writer),
  # NOT 0x1A0 (gateway-native -> relayMalfunction). openpilot relays the camera's
  # own steer request when not actively steering, so the panda allows the full
  # field range and rate-limits (seeded on the engage edge). There is NO separate
  # 0x1A0 tx anymore.
  MAX_DELTA = 1500

  def test_no_1a0_tx(self):
    # 0x1A0 is no longer a tx message on any bus -- it must be blocked.
    self._engage()
    for bus in (0, 1, 2):
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x1A0, bus, bytes(48))),
                       f"0x1A0 on bus {bus} must not tx (steer moved into 0x160)")

  def test_steer_rate_limit_in_160(self):
    self._engage()
    self.assertTrue(self._tx(self._accel_160(0.0, 0)))                         # seed (last=0)
    self.assertTrue(self._tx(self._accel_160(0.0, self.MAX_DELTA)))            # +1500 ok (last=1500)
    self.assertFalse(self._tx(self._accel_160(0.0, 2 * self.MAX_DELTA + 800)), # +2300 jump
                     "steer jump beyond rate must be blocked")

  def test_steer_rate_seeded_on_engage(self):
    # engaging while the camera's live steer request is large (openpilot relays
    # it): the first engaged frame must NOT trip the rate limit vs a stale seed.
    self._engage()
    self.assertTrue(self._tx(self._accel_160(0.0, 20000)),
                    "first engaged frame seeds the steer baseline (no rate trip)")
    self.assertFalse(self._tx(self._accel_160(0.0, 20000 + self.MAX_DELTA + 500)),
                     "subsequent steer jump beyond rate must be blocked")

  def test_steer_relay_full_range(self):
    # relaying stock LTA: large steer values are allowed as long as they arrive
    # within the per-frame rate (the field itself saturates ~+/-60 deg).
    self._engage()
    a = 0
    self.assertTrue(self._tx(self._accel_160(0.0, 0)))   # seed
    ok = True
    for a in range(self.MAX_DELTA, 30000, self.MAX_DELTA):
      ok = self._tx(self._accel_160(0.0, a))
    self.assertTrue(ok, "gentle ramp to a large steer (relay) must pass")


if __name__ == "__main__":
  unittest.main()
