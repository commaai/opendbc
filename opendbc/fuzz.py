"""Small, deterministic property-test generator used by openpilot tests.

This intentionally implements only the generators we use.  Values are biased
toward boundaries, while still sampling the full requested range.
"""

import functools
import hashlib
import inspect
import math
import random
import struct
from collections.abc import Callable, Sequence
from typing import Any, Generic, TypeVar

T = TypeVar("T")


class Strategy(Generic[T]):
  def __init__(self, generate: Callable[[random.Random, int], T]):
    self._generate = generate

  def example(self, rng: random.Random, index: int = 0) -> T:
    return self._generate(rng, index)

class Data:
  def __init__(self, rng: random.Random, index: int):
    self.rng, self.index, self.draws = rng, index, 0

  def draw(self, strategy: Strategy[T]) -> T:
    value = strategy.example(self.rng, self.index + self.draws)
    self.draws += 1
    return value


def _biased_int(rng: random.Random, index: int, low: int, high: int) -> int:
  edges = (low, high, 0, 1, -1, low + 1, high - 1)
  valid = tuple(v for v in edges if low <= v <= high)
  return valid[index % len(valid)] if index < len(valid) else rng.randint(low, high)


def booleans() -> Strategy[bool]:
  return Strategy(lambda _rng, i: bool(i % 2))


def integers(min_value: int | None = None, max_value: int | None = None) -> Strategy[int]:
  low = -(2 ** 63) if min_value is None else min_value
  high = 2 ** 63 - 1 if max_value is None else max_value
  return Strategy(lambda rng, i: _biased_int(rng, i, low, high))


def sampled_from(values: Sequence[T]) -> Strategy[T]:
  values = tuple(values)
  if not values:
    raise ValueError("sampled_from requires at least one value")
  return Strategy(lambda rng, i: values[0] if i == 0 else values[-1] if i == 1 else rng.choice(values))


def binary(min_size: int = 0, max_size: int | None = None) -> Strategy[bytes]:
  max_size = 64 if max_size is None else max_size

  def generate(rng: random.Random, index: int) -> bytes:
    useful_sizes = tuple(size for size in (min_size, max_size, 0, 1, 8, 10, 12, 16, 20, 24, 32, 48, 64)
                         if min_size <= size <= max_size)
    size = useful_sizes[index] if index < len(useful_sizes) else rng.randint(min_size, max_size)
    patterns = (b"\x00", b"\xff", b"?", b"!")
    if index < len(patterns):
      return (patterns[index] * size)[:size]
    return rng.randbytes(size)
  return Strategy(generate)


def text(min_size: int = 0, max_size: int | None = None) -> Strategy[str]:
  max_size = 10 if max_size is None else max_size
  alphabet = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\x00"
  return Strategy(lambda rng, i: "".join(rng.choice(alphabet) for _ in range(_biased_int(rng, i, min_size, max_size))))


def floats(width: int = 64, allow_nan: bool = True, allow_infinity: bool = True) -> Strategy[float]:
  finite_edges = [0.0, -0.0, 1.0, -1.0, float.fromhex("0x0.0000000000001p-1022")]
  edges = finite_edges + ([math.inf, -math.inf] if allow_infinity else []) + ([math.nan] if allow_nan else [])
  fmt = "!I" if width == 32 else "!Q"
  unpack_fmt = "!f" if width == 32 else "!d"
  bits = 32 if width == 32 else 64

  def generate(rng: random.Random, index: int) -> float:
    if index < len(edges):
      return edges[index]
    value = struct.unpack(unpack_fmt, struct.pack(fmt, rng.getrandbits(bits)))[0]
    if (not allow_nan and math.isnan(value)) or (not allow_infinity and math.isinf(value)):
      return 0.0
    return value
  return Strategy(generate)


def lists(elements: Strategy[T], min_size: int = 0, max_size: int | None = None) -> Strategy[list[T]]:
  max_size = min_size + 5 if max_size is None else max_size
  return Strategy(lambda rng, i: [elements.example(rng, i + n) for n in range(_biased_int(rng, i, min_size, max_size))])


def dictionaries(keys: Strategy, values: Strategy, min_size: int = 0, max_size: int | None = None) -> Strategy[dict]:
  max_size = max(10, min_size + 10) if max_size is None else max_size

  def generate(rng: random.Random, index: int) -> dict:
    target = _biased_int(rng, index, min_size, max_size)
    result = {}
    for n in range(target * 4 + 10):
      result[keys.example(rng, index + n)] = values.example(rng, index + n)
      if len(result) >= target:
        break
    return result
  return Strategy(generate)


def fixed_dictionaries(mapping: dict[Any, Strategy]) -> Strategy[dict]:
  return Strategy(lambda rng, i: {key: strategy.example(rng, i + n) for n, (key, strategy) in enumerate(mapping.items())})


def builds(target: Callable, *strategies: Strategy, **kw_strategies: Strategy) -> Strategy:
  return Strategy(lambda rng, i: target(*(s.example(rng, i + n) for n, s in enumerate(strategies)),
                                        **{k: s.example(rng, i + len(strategies) + n)
                                           for n, (k, s) in enumerate(kw_strategies.items())}))


def data() -> Strategy[Data]:
  return Strategy(lambda rng, i: Data(rng, i))


def fuzz(*, examples: int = 100, **strategies: Strategy):
  """Run a test repeatedly with deterministic values from named strategies."""
  def decorator(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
      instance = args[0] if args else None
      test_name = getattr(instance, "_testMethodName", "")
      identity = f"{func.__module__}.{func.__qualname__}.{test_name}".encode()
      base_seed = int.from_bytes(hashlib.sha256(identity).digest()[:8])
      for index in range(examples):
        rng = random.Random(base_seed + index)
        generated = {name: strategy.example(rng, index + n) for n, (name, strategy) in enumerate(strategies.items())}
        func(*args, **kwargs, **generated)
    # Generated arguments must not be interpreted as pytest fixtures.
    signature = inspect.signature(func)
    parameters = [p for p in signature.parameters.values() if p.name not in strategies]
    wrapper.__signature__ = signature.replace(parameters=parameters)
    return wrapper
  return decorator
