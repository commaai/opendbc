#!/usr/bin/env python3
import unittest

from opendbc.car.byd.cam_lka.bydcan import create_steering_torque_spoof_camera
from opendbc.car.byd.values import CAR, DBC
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.car.structs import CarParams


class TestAtto3TorqueSpoof(unittest.TestCase):
  def test_only_param1_blocks_pt_torque(self):
    s = libsafety_py.libsafety
    # Default / unset: forward stock 0x1FC (other BYDs must not inherit Atto block).
    s.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.assertEqual(2, s.safety_fwd_hook(0, 0x1FC))

    # Atto (param 1): block PT→cam torque.
    s.set_safety_hooks(CarParams.SafetyModel.byd, 1)
    self.assertEqual(-1, s.safety_fwd_hook(0, 0x1FC))

    # Seal-style (param 2) and M6 (param 3) already spoofed before Atto; still block.
    s.set_safety_hooks(CarParams.SafetyModel.byd, 2)
    self.assertEqual(-1, s.safety_fwd_hook(0, 0x1FC))
    s.set_safety_hooks(CarParams.SafetyModel.byd, 3)
    self.assertEqual(-1, s.safety_fwd_hook(0, 0x1FC))

    # Song Plus MPC LKA (param 4): no torque spoof.
    s.set_safety_hooks(CarParams.SafetyModel.byd, 4)
    self.assertEqual(2, s.safety_fwd_hook(0, 0x1FC))

  def test_spoof_payload_holds_nonzero_torque(self):
    # create_steering_torque_spoof_camera's ramp target re-randomizes (and can
    # briefly dip through zero on a sign flip) once it settles, so this checks
    # the peak reached over the window rather than one non-deterministic call.
    packer = CANPacker(DBC[CAR.BYD_ATTO3]["pt"])
    parser = CANParser(DBC[CAR.BYD_ATTO3]["pt"], [("STEERING_TORQUE", 0)], 2)
    peak_torque = 0.0
    for i in range(10):
      msg = create_steering_torque_spoof_camera(packer, True, 0.0, True)
      parser.update([(i, [(msg[0], msg[1], 2)])])
      torque = parser.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
      peak_torque = max(peak_torque, abs(torque))
    self.assertGreaterEqual(peak_torque, 5.0)


if __name__ == "__main__":
  unittest.main()
