import unittest
from opendbc.car.honda.acura import AcuraILX2016CAN

class TestAcuraILX2016CAN(unittest.TestCase):
    def setUp(self):
        self.can = AcuraILX2016CAN()

    def test_update(self):
        can_messages = [
            {
                "name": "STEERING_LKA",
                "signals": [
                    {"name": "STEERING_ANGLE", "value": 420},
                    {"name": "STEERING_RATE", "value": 100},
                ],
            },
            {
                "name": "BRAKE_COMMAND",
                "signals": [{"name": "BRAKE_PRESSED", "value": 1}],
            },
        ]

        signals = self.can.update(can_messages)
        self.assertEqual(signals["STEERING_ANGLE"], 42.0)
        self.assertEqual(signals["STEERING_RATE"], 1.0)
        self.assertEqual(signals["BRAKE_PRESSED"], 1)

    def test_check_messages(self):
        signals = {
            "STEERING_ANGLE": 42.0,
            "STEERING_RATE": 1.0,
            "BRAKE_PRESSED": 1,
            "GAS_PRESSED": 0,
        }
        checks = self.can.check_messages(signals)
        self.assertEqual(checks, {
            "STEERING_ANGLE": 42.0,
            "STEERING_RATE": 1.0,
            "BRAKE_PRESSED": 1,
        })