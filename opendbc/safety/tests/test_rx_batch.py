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
      state = ffi.new("SafetyState *")
      safety.get_safety_state(state)
      getters = {
        "gas_pressed": safety.get_gas_pressed_prev,
        "brake_pressed": safety.get_brake_pressed_prev,
        "regen_braking": safety.get_regen_braking_prev,
        "steering_disengage": safety.get_steering_disengage_prev,
        "vehicle_moving": safety.get_vehicle_moving,
        "controls_allowed": safety.get_controls_allowed,
        "cruise_engaged": safety.get_cruise_engaged_prev,
        "acc_main_on": safety.get_acc_main_on,
        "vehicle_speed_min": safety.get_vehicle_speed_min,
        "vehicle_speed_max": safety.get_vehicle_speed_max,
        "angle_min": safety.get_angle_meas_min,
        "angle_max": safety.get_angle_meas_max,
      }
      for field, getter in getters.items():
        self.assertEqual(getattr(state, field), getter(), field)
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
