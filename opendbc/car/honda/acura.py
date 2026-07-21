from opendbc.can.parser import CANParser

class AcuraILX2016CAN(CANParser):
    def __init__(self):
        super().__init__(
            dbc_file="honda_ilx_2016.dbc",
            messages=[
                "STEERING_LKA",
                "STEERING_CONTROL",
                "STEERING_STATUS",
                "BRAKE_COMMAND",
                "BRAKE_STATUS",
                "GAS_COMMAND",
                "GAS_STATUS",
            ],
            checks=[
                "STEERING_ANGLE",
                "STEERING_RATE",
                "BRAKE_PRESSED",
                "GAS_PRESSED",
            ],
        )

    def update(self, can_messages):
        signals = super().update(can_messages)
        if "STEERING_ANGLE" in signals:
            signals["STEERING_ANGLE"] = signals["STEERING_ANGLE"] * 0.1
        if "STEERING_RATE" in signals:
            signals["STEERING_RATE"] = signals["STEERING_RATE"] * 0.01
        return signals