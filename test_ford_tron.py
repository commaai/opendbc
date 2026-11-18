import unittest
import cantools
from hypothesis import given, strategies as st
from ford_parser import FordTronParser

class TestFordTronArchitecture(unittest.TestCase):
    def setUp(self):
        self.parser = FordTronParser('ford_tron_2026.dbc')

    def test_steering_decode(self):
        # Mock payload representing 45.0 degrees and 1.5 Nm torque
        mock_data = b'\x22\01\x96\00\00\00\00\00'
        res = self.parser.decode_steering(mock_data)
        self.assertIn('STEER_ANGLE', res)
        self.assertIn('STEER_TORQUE', res)

    @given(gas=st.integers(min_value=0, max_value=255))
    def test_gas_pedal_fuzzing(self, gas):
        # Fuzzing longitudinal payloads to ensure no struct unpack crashes
        payload = bytes([gas, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        try:
            self.parser.decode_longitudinal(payload)
        except cantools.database.errors.DecodeError:
            pass # Expected for out-of-bound fuzzy data, but should not crash the python runtime

if __name__ == '__main__':
    unittest.main()
