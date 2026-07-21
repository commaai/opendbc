import unittest
from opendbc.can.parser import CANParser

class TestCANParser(unittest.TestCase):
    def setUp(self):
        self.parser = CANParser(
            dbc_file="test.dbc",
            messages=["TEST_MESSAGE"],
            checks=["TEST_SIGNAL"],
        )

    def test_update(self):
        can_messages = [
            {
                "name": "TEST_MESSAGE",
                "signals": [
                    {"name": "TEST_SIGNAL", "value": 42},
                    {"name": "OTHER_SIGNAL", "value": 100},
                ],
            },
            {
                "name": "OTHER_MESSAGE",
                "signals": [{"name": "TEST_SIGNAL", "value": 99}],
            },
        ]

        signals = self.parser.update(can_messages)
        self.assertEqual(signals, {"TEST_SIGNAL": 42})

    def test_check_messages(self):
        signals = {"TEST_SIGNAL": 42, "OTHER_SIGNAL": 100}
        checks = self.parser.check_messages(signals)
        self.assertEqual(checks, {"TEST_SIGNAL": 42})

    def test_check_messages_missing(self):
        signals = {"OTHER_SIGNAL": 100}
        checks = self.parser.check_messages(signals)
        self.assertEqual(checks, {})