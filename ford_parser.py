import os
import re


def parse_dbc_file(filepath: str) -> dict:
    """Parse DBC messages and signals using pure Python standard library."""
    messages = {}
    current_msg = None
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"DBC file not found: {filepath}")

    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            msg_match = re.match(r"^\s*BO_\s+(\d+)\s+(\w+):\s+(\d+)\s+(\w+)", line)
            if msg_match:
                current_msg = msg_match.group(2)
                messages[current_msg] = {
                    "id": int(msg_match.group(1)),
                    "length": int(msg_match.group(3)),
                    "sender": msg_match.group(4),
                    "signals": [],
                }
                continue
            sig_match = re.match(r"^\s*SG_\s+(\w+)\s*:", line)
            if sig_match and current_msg:
                messages[current_msg]["signals"].append(sig_match.group(1))
    return messages
