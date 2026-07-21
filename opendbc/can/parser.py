from typing import Optional, Dict, Any
import can

class CANParser:
    def __init__(self, dbc_file: str, messages: list, checks: Optional[list] = None):
        self.dbc = can.DBC(dbc_file)
        self.messages = messages
        self.checks = checks or []
        self.signal_names = set()

        for message in messages:
            for signal in self.dbc.get_message_by_name(message).signals:
                self.signal_names.add(signal.name)

    def update(self, can_messages: list) -> Dict[str, Any]:
        signals = {}
        for message in can_messages:
            if message.name in self.messages:
                for signal in message.signals:
                    if signal.name in self.signal_names:
                        signals[signal.name] = signal.value
        return signals

    def check_messages(self, signals: Dict[str, Any]) -> Dict[str, Any]:
        checks = {}
        for check in self.checks:
            if check in signals:
                checks[check] = signals[check]
        return checks