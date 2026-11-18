import os
import unittest
from ford_parser import parse_dbc_file


class TestFordTronDBC(unittest.TestCase):
    def setUp(self):
        self.dbc_path = os.path.join(os.path.dirname(__file__), "ford_tron_2026.dbc")
        self.messages = parse_dbc_file(self.dbc_path)

    def test_steering_message_and_safety_signals(self):
        self.assertTrue("STEERING_DATA" in self.messages)
        signals = self.messages["STEERING_DATA"]["signals"]
        for required in ("STEER_ANGLE", "STEER_TORQUE", "COUNTER", "CHECKSUM"):
            self.assertTrue(required in signals)

    def test_longitudinal_message_and_safety_signals(self):
        self.assertTrue("LONGITUDINAL_DATA" in self.messages)
        signals = self.messages["LONGITUDINAL_DATA"]["signals"]
        for required in ("GAS_PEDAL", "BRAKE_PRESSURE", "COUNTER", "CHECKSUM"):
            self.assertTrue(required in signals)


if __name__ == "__main__":
    unittest.main()
