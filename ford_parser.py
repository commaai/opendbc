import os
import re


def parse_dbc_file(filepath: str) -> dict[str, list[str]]:
  """Parse DBC message names and their signal lists using stdlib only."""
  messages: dict[str, list[str]] = {}
  current_msg: str | None = None
  if not os.path.exists(filepath):
    raise FileNotFoundError(filepath)

  with open(filepath, encoding="utf-8") as f:
    for line in f:
      msg_match = re.match(r"^\s*BO_\s+(\d+)\s+(\w+):\s+(\d+)\s+(\w+)", line)
      if msg_match:
        current_msg = msg_match.group(2)
        messages[current_msg] = []
        continue
      sig_match = re.match(r"^\s*SG_\s+(\w+)\s*:", line)
      if sig_match and current_msg is not None:
        messages[current_msg].append(sig_match.group(1))
  return messages
