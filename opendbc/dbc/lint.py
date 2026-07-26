"""Small, dependency-free checks for common DBC authoring mistakes."""

import argparse
import re
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path


BO_RE = re.compile(r"^BO_\s+(?P<address>\S+)\s+(?P<name>\S+)\s*:\s*(?P<size>\d+)\s+\S+")
SG_RE = re.compile(r"^SG_\s+(?P<name>\S+)(?:\s+(?P<multiplexer>M|m\d+M?))?\s*:\s*(?P<start>\d+)\|(?P<size>\d+)@(?P<byte_order>[01])[+-]")


@dataclass(frozen=True)
class LintError:
  path: Path
  line: int
  message: str

  def __str__(self) -> str:
    return f"{self.path}:{self.line}: error: {self.message}"


@dataclass
class Signal:
  name: str
  line: int
  bits: set[int]
  multiplexer: str | None


@dataclass
class Message:
  name: str
  size: int
  line: int
  signals: list[Signal]


def _signal_bits(start: int, size: int, byte_order: str, message_size: int) -> set[int] | None:
  if byte_order == "1":
    bits = set(range(start, start + size))
  else:
    # Motorola signals use the DBC "sawtooth" bit order: 7, 6, ..., 0, 15, ...
    sawtooth_bits = [byte * 8 + bit for byte in range(message_size) for bit in range(7, -1, -1)]
    if start not in sawtooth_bits:
      return None
    start_index = sawtooth_bits.index(start)
    bits = set(sawtooth_bits[start_index:start_index + size])

  return bits if len(bits) == size and max(bits, default=-1) < message_size * 8 else None


def _mutually_exclusive(first: Signal, second: Signal) -> bool:
  """Old-style multiplexed signals with different selectors may share bits."""
  return (
    first.multiplexer is not None
    and second.multiplexer is not None
    and first.multiplexer.startswith("m")
    and second.multiplexer.startswith("m")
    and first.multiplexer.rstrip("M") != second.multiplexer.rstrip("M")
  )


def lint_file(path: str | Path) -> list[LintError]:
  """Return layout errors found in one DBC file."""
  path = Path(path)
  errors: list[LintError] = []
  messages_by_address: dict[int, Message] = {}
  messages_by_name: dict[str, Message] = {}
  message: Message | None = None

  for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
    line = line.strip()
    if match := BO_RE.match(line):
      address = int(match["address"], 0)
      name = match["name"]
      size = int(match["size"])
      if previous := messages_by_address.get(address):
        errors.append(LintError(path, line_number, f"message address {address} is already used by {previous.name} (line {previous.line})"))
      if previous := messages_by_name.get(name):
        errors.append(LintError(path, line_number, f"message name {name} is already defined on line {previous.line}"))
      message = Message(name, size, line_number, [])
      messages_by_address[address] = message
      messages_by_name[name] = message
      continue

    if not (match := SG_RE.match(line)) or message is None:
      continue

    name = match["name"]
    duplicate_name = any(previous.name == name for previous in message.signals)
    if duplicate_name:
      errors.append(LintError(path, line_number, f"signal {name} is already defined in {message.name}"))

    # Vector uses this zero-length pseudo-message to declare independent signals.
    # It has no CAN payload, so normal layout checks do not apply to it.
    if message.size == 0:
      continue

    size = int(match["size"])
    if size == 0:
      errors.append(LintError(path, line_number, f"signal {name} has zero length"))
      continue

    bits = _signal_bits(int(match["start"]), size, match["byte_order"], message.size)
    if bits is None:
      errors.append(LintError(path, line_number, f"signal {name} extends outside {message.name}'s {message.size}-byte payload"))
      continue

    signal = Signal(name, line_number, bits, match["multiplexer"])
    for previous in message.signals:
      overlapping_bits = signal.bits & previous.bits
      if overlapping_bits and not _mutually_exclusive(signal, previous):
        rendered_bits = ", ".join(map(str, sorted(overlapping_bits)))
        errors.append(LintError(path, line_number, f"signal {name} overlaps {previous.name} (line {previous.line}) at bit(s) {rendered_bits}"))

    message.signals.append(signal)

  return errors


def _dbc_files(paths: Iterable[str | Path]) -> list[Path]:
  files: set[Path] = set()
  for path_arg in paths:
    path = Path(path_arg)
    if path.is_dir():
      files.update(path.rglob("*.dbc"))
    elif path.suffix == ".dbc":
      files.add(path)
  return sorted(files)


def lint_paths(paths: Iterable[str | Path]) -> list[LintError]:
  return [error for path in _dbc_files(paths) for error in lint_file(path)]


def main() -> int:
  parser = argparse.ArgumentParser(description="Lint DBC files for common layout mistakes.")
  parser.add_argument("paths", nargs="*", default=["opendbc/dbc"], help="DBC files or directories to lint")
  args = parser.parse_args()
  errors = lint_paths(args.paths)
  for error in errors:
    print(error)
  return int(bool(errors))


if __name__ == "__main__":
  raise SystemExit(main())
