import unittest

from opendbc.can import CANPacker
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class TestRxBatch(unittest.TestCase):
  def test_replay_matches_individual_hooks(self):
    safety = libsafety_py.libsafety
    ffi = libsafety_py.ffi
    packer = CANPacker("honda_civic_touring_2016_can_generated")
    packets = []
    for i in range(20):
      addr, data, bus = packer.make_can_msg("POWERTRAIN_DATA", 0, {
        "ACC_STATUS": i % 2, "PEDAL_GAS": i % 3, "BRAKE_PRESSED": i % 4 == 0,
      })
      if i == 10:
        data = data[:-1] + bytes([data[-1] ^ 1])
      packets.append(libsafety_py.make_CANPacket(addr, bus, data))

    def reset():
      self.assertEqual(safety.set_safety_hooks(CarParams.SafetyModel.hondaNidec, 0), 0)
      safety.init_tests()

    def snapshot():
      return (safety.get_controls_allowed(), safety.get_gas_pressed_prev(), safety.get_brake_pressed_prev(),
              safety.get_cruise_engaged_prev(), safety.get_relay_malfunction(), safety.safety_config_valid())

    reset()
    expected = [bool(safety.safety_rx_hook(packet)) for packet in packets]
    expected_state = snapshot()
    self.assertFalse(all(expected))
    self.assertTrue(any(expected))
    pointers = ffi.new("CANPacket_t *[]", packets)
    valid = ffi.new("bool[]", len(packets))
    reset()
    self.assertEqual(safety.safety_rx_hook_batch(pointers, len(packets), valid), expected.count(False))
    self.assertEqual(list(valid), expected)
    self.assertEqual(snapshot(), expected_state)
    reset()
    self.assertEqual(safety.safety_rx_hook_batch(pointers, len(packets), ffi.NULL), expected.count(False))
    self.assertEqual(snapshot(), expected_state)
    self.assertEqual(safety.safety_rx_hook_batch(pointers, 0, ffi.NULL), 0)
    self.assertEqual(snapshot(), expected_state)
