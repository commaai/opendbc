from __future__ import annotations

import dataclasses
import functools
import itertools
import os
import random
from collections.abc import Callable, Iterable, Mapping, Sequence
from enum import Enum
from typing import Any, Generic, TypeVar

T = TypeVar("T")

_BINARY_LENGTHS = (0, 1, 2, 3, 7, 8, 10, 12, 15, 16, 17, 31, 32, 47, 48, 64, 65)
_COLLECTION_SIZES = (0, 1, 2, 3, 4, 8, 16, 24, 32)
_MAX_COLLECTION_SIZE = 32
_DEFAULT_MAX_EXAMPLES = 100
_DEFAULT_SEED = int(os.environ.get("PROPTEST_SEED", "0"))
_CURATED_INTS = (
  0x126, 0x184, 0x391, 0x3F6, 0x6B8, 0x40, 0x50, 0x86, 0xB2, 0xFD, 0x5A, 0xAD, 0x187, 0x191, 0x1A3, 0x1E5, 0x30F,
  0x3A6, 0x3A7, 0x3BA, 0x58B, 0, 1, 0x800,
)


def _dedupe(values: Iterable[T]) -> list[T]:
  seen = set()
  unique = []
  for value in values:
    marker = repr(value)
    if marker in seen:
      continue
    seen.add(marker)
    unique.append(value)
  return unique


def _clone(value: T) -> T:
  if isinstance(value, dict):
    return {k: _clone(v) for k, v in value.items()}
  if isinstance(value, list):
    return [_clone(item) for item in value]
  if isinstance(value, tuple):
    return tuple(_clone(item) for item in value)
  return value


def _clamp(value: int, min_value: int | None, max_value: int | None) -> int:
  if min_value is not None:
    value = max(min_value, value)
  if max_value is not None:
    value = min(max_value, value)
  return value


def _coerce_exception(exc: BaseException, example: Mapping[str, Any]) -> BaseException:
  details = f"Falsifying example: {', '.join(f'{k}={v!r}' for k, v in example.items())}"
  message = str(exc)
  if message:
    message = f"{message}\n{details}"
  else:
    message = details
  return type(exc)(message)


class Phase(Enum):
  reuse = "reuse"
  generate = "generate"
  shrink = "shrink"


@dataclasses.dataclass(frozen=True)
class Settings:
  max_examples: int = _DEFAULT_MAX_EXAMPLES
  deadline: int | None = None
  phases: tuple[Phase, ...] = (Phase.generate, Phase.shrink)


DEFAULT_SETTINGS = Settings()


class SearchStrategy(Generic[T]):
  def edge_cases(self) -> list[T]:
    return []

  def generate(self, rng: random.Random) -> T:
    raise NotImplementedError

  def shrink(self, value: T) -> Iterable[T]:
    return ()

  def examples(self, max_examples: int, seed: int) -> list[T]:
    examples = self.edge_cases()
    remaining = max(0, max_examples - len(examples))
    for i in range(remaining):
      examples.append(self.generate(random.Random(seed + i)))
    return examples[:max_examples]


class IntegerStrategy(SearchStrategy[int]):
  def __init__(self, min_value: int | None = None, max_value: int | None = None):
    self.min_value = min_value
    self.max_value = max_value

  def edge_cases(self) -> list[int]:
    candidates = [
      *_CURATED_INTS,
      self.min_value,
      None if self.min_value is None else self.min_value + 1,
      -1,
      None if self.max_value is None else self.max_value - 1,
      self.max_value,
    ]
    values = [
      _clamp(value, self.min_value, self.max_value)
      for value in candidates
      if value is not None
    ]
    return _dedupe(values)

  def generate(self, rng: random.Random) -> int:
    if self.min_value is not None and self.max_value is not None:
      return rng.randint(self.min_value, self.max_value)

    span = 1 << rng.randint(0, 16)
    value = rng.randint(-span, span)
    return _clamp(value, self.min_value, self.max_value)

  def shrink(self, value: int) -> Iterable[int]:
    targets = [
      self.min_value,
      0,
      None if self.max_value is None else self.max_value,
      value // 2,
      1 if value > 0 else -1 if value < 0 else 0,
    ]
    return _dedupe(
      _clamp(candidate, self.min_value, self.max_value)
      for candidate in targets
      if candidate is not None and candidate != value
    )


class SampledFromStrategy(SearchStrategy[T]):
  def __init__(self, values: Sequence[T]):
    if not values:
      raise ValueError("sampled_from() requires at least one value")
    self.values = tuple(values)

  def edge_cases(self) -> list[T]:
    if len(self.values) <= 7:
      return list(self.values)
    positions = [0, 1, 2, 3, 4, len(self.values) // 2, len(self.values) - 1]
    return _dedupe(self.values[position] for position in positions)

  def generate(self, rng: random.Random) -> T:
    return self.values[rng.randrange(len(self.values))]


class BooleanStrategy(SampledFromStrategy[bool]):
  def __init__(self):
    super().__init__((False, True))


class BinaryStrategy(SearchStrategy[bytes]):
  def __init__(self, min_size: int = 0, max_size: int | None = None):
    self.min_size = min_size
    self.max_size = max_size

  def _bounded_length(self, length: int) -> int:
    return _clamp(length, self.min_size, self.max_size)

  def edge_cases(self) -> list[bytes]:
    values = []
    for length in _BINARY_LENGTHS:
      length = self._bounded_length(length)
      patterns = [
        b"\x00" * length,
        b"\xff" * length,
        (b"A" * length),
      ]
      if length > 0:
        patterns.extend((
          bytes([1]) + (b"\x00" * (length - 1)),
          bytes([2]) + (b"A" * (length - 1)),
          bytes([3]) + (b" " * (length - 1)),
        ))
      values.extend(patterns)
    return _dedupe(values)

  def generate(self, rng: random.Random) -> bytes:
    lengths = [self._bounded_length(length) for length in _BINARY_LENGTHS]
    max_length = self._bounded_length(_MAX_COLLECTION_SIZE * 2)
    if max_length not in lengths:
      lengths.append(max_length)
    length = rng.choice(lengths) if rng.random() < 0.7 else rng.randint(self.min_size, max_length)

    mode = rng.randrange(5)
    if mode == 0:
      return b"\x00" * length
    if mode == 1:
      return b"\xff" * length
    if mode == 2:
      return bytes(rng.randrange(256) for _ in range(length))
    if mode == 3:
      alphabet = b"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 -_"
      return bytes(alphabet[rng.randrange(len(alphabet))] for _ in range(length))

    if length == 0:
      return b""
    prefix = bytes([rng.choice((1, 2, 3))])
    return prefix + rng.randbytes(max(0, length - 1))

  def shrink(self, value: bytes) -> Iterable[bytes]:
    candidates = [b""]
    for length in (0, len(value) // 2, max(0, len(value) - 1)):
      candidates.append(value[:length])
      candidates.append(b"\x00" * length)
    return _dedupe(candidate for candidate in candidates if candidate != value)


class ListStrategy(SearchStrategy[list[T]]):
  def __init__(self, element_strategy: SearchStrategy[T], min_size: int = 0, max_size: int | None = None):
    self.element_strategy = element_strategy
    self.min_size = min_size
    self.max_size = max_size

  def _sizes(self) -> list[int]:
    sizes = [_clamp(size, self.min_size, self.max_size) for size in _COLLECTION_SIZES]
    max_size = _clamp(_MAX_COLLECTION_SIZE, self.min_size, self.max_size)
    if max_size not in sizes:
      sizes.append(max_size)
    return _dedupe(sizes)

  def edge_cases(self) -> list[list[T]]:
    values = []
    child_edges = self.element_strategy.edge_cases()
    if self.min_size == 0:
      values.append([])
    if child_edges:
      first = child_edges[0]
      values.append([first])
      if len(child_edges) > 1:
        second = child_edges[1]
        values.append([second])
        values.append([first, second])
        values.append([second, first])
      values.append([first] * max(1, self.min_size))
      values.append([first, first])
      for child in child_edges[2:5]:
        values.append([child])
      for size in self._sizes()[1:4]:
        values.append([child_edges[i % len(child_edges)] for i in range(size)])
    return [value for value in _dedupe(values) if len(value) >= self.min_size]

  def generate(self, rng: random.Random) -> list[T]:
    sizes = self._sizes()
    max_size = sizes[-1]
    size = rng.choice(sizes) if rng.random() < 0.7 else rng.randint(self.min_size, max_size)
    return [self.element_strategy.generate(rng) for _ in range(size)]

  def shrink(self, value: list[T]) -> Iterable[list[T]]:
    if not value:
      return ()

    candidates = [[]]
    if len(value) > 1:
      candidates.extend((value[:len(value) // 2], value[:-1], value[1:]))

    for i, item in enumerate(value):
      for shrunk in itertools.islice(self.element_strategy.shrink(item), 4):
        candidate = list(value)
        candidate[i] = shrunk
        candidates.append(candidate)

    return [candidate for candidate in _dedupe(candidates) if len(candidate) >= self.min_size and candidate != value]


class DictStrategy(SearchStrategy[dict[Any, Any]]):
  def __init__(self, key_strategy: SearchStrategy[Any], value_strategy: SearchStrategy[Any], min_size: int = 0, max_size: int | None = None):
    self.key_strategy = key_strategy
    self.value_strategy = value_strategy
    self.min_size = min_size
    self.max_size = max_size

  def edge_cases(self) -> list[dict[Any, Any]]:
    key_edges = self.key_strategy.edge_cases()
    value_edges = self.value_strategy.edge_cases()
    values: list[dict[Any, Any]] = []
    if self.min_size == 0:
      values.append({})
    if key_edges and value_edges:
      for key in key_edges[:8]:
        values.append({key: value_edges[0]})
      if len(value_edges) > 1:
        for key in key_edges[:4]:
          values.append({key: value_edges[1]})
      if len(key_edges) > 1:
        values.append({key_edges[0]: value_edges[0], key_edges[1]: value_edges[min(1, len(value_edges) - 1)]})
      if len(key_edges) > 3:
        values.append({key_edges[i]: value_edges[i % len(value_edges)] for i in range(4)})
    return [value for value in _dedupe(values) if len(value) >= self.min_size]

  def generate(self, rng: random.Random) -> dict[Any, Any]:
    max_size = _clamp(_MAX_COLLECTION_SIZE, self.min_size, self.max_size)
    size = rng.choice([_clamp(size, self.min_size, self.max_size) for size in _COLLECTION_SIZES])
    if rng.random() >= 0.7:
      size = rng.randint(self.min_size, max_size)

    result = {}
    while len(result) < size:
      result[self.key_strategy.generate(rng)] = self.value_strategy.generate(rng)
    return result

  def shrink(self, value: dict[Any, Any]) -> Iterable[dict[Any, Any]]:
    if not value:
      return ()

    items = list(value.items())
    candidates = [{}]
    if len(items) > 1:
      candidates.append(dict(items[:len(items) // 2]))
      candidates.append(dict(items[:-1]))

    for key, item in items:
      for shrunk in itertools.islice(self.value_strategy.shrink(item), 4):
        candidate = dict(items)
        candidate[key] = shrunk
        candidates.append(candidate)
      candidate = dict(items)
      del candidate[key]
      candidates.append(candidate)

    return [candidate for candidate in _dedupe(candidates) if len(candidate) >= self.min_size and candidate != value]


class FixedDictionariesStrategy(SearchStrategy[dict[Any, Any]]):
  def __init__(self, mapping: Mapping[Any, SearchStrategy[Any]]):
    self.mapping = dict(mapping)

  def edge_cases(self) -> list[dict[Any, Any]]:
    base = {}
    edge_values = {}
    for key, strategy in self.mapping.items():
      values = strategy.edge_cases()
      edge_values[key] = values
      base[key] = _clone(values[0] if values else strategy.generate(random.Random(_DEFAULT_SEED)))

    cases = [_clone(base)]
    max_variants = max((len(values) for values in edge_values.values()), default=0)
    for index in range(1, min(max_variants, 9)):
      for key, values in edge_values.items():
        if index >= len(values):
          continue
        candidate = _clone(base)
        candidate[key] = _clone(values[index])
        cases.append(candidate)
    return _dedupe(cases)

  def generate(self, rng: random.Random) -> dict[Any, Any]:
    return {key: strategy.generate(rng) for key, strategy in self.mapping.items()}

  def shrink(self, value: dict[Any, Any]) -> Iterable[dict[Any, Any]]:
    candidates = []
    for key, strategy in self.mapping.items():
      current = value[key]
      for shrunk in itertools.islice(strategy.shrink(current), 4):
        candidate = _clone(value)
        candidate[key] = shrunk
        candidates.append(candidate)
    return _dedupe(candidates)


class BuildsStrategy(SearchStrategy[T]):
  def __init__(self, fn: Callable[..., T], strategies: Sequence[SearchStrategy[Any]]):
    self.fn = fn
    self.strategies = tuple(strategies)

  def _base_args(self) -> list[Any]:
    args = []
    for strategy in self.strategies:
      edges = strategy.edge_cases()
      args.append(_clone(edges[0] if edges else strategy.generate(random.Random(_DEFAULT_SEED))))
    return args

  def edge_cases(self) -> list[T]:
    base = self._base_args()
    cases = [self.fn(*_clone(base))]
    for i, strategy in enumerate(self.strategies):
      for edge in strategy.edge_cases()[1:5]:
        args = _clone(base)
        args[i] = _clone(edge)
        cases.append(self.fn(*args))
    return cases

  def generate(self, rng: random.Random) -> T:
    return self.fn(*(strategy.generate(rng) for strategy in self.strategies))


def _get_settings(func: Callable[..., Any]) -> Settings:
  return getattr(func, "_propcheck_settings", DEFAULT_SETTINGS)


def settings(*, max_examples: int | None = None, deadline: int | None = None, phases: Sequence[Phase] | None = None):
  configured = dataclasses.replace(
    DEFAULT_SETTINGS,
    max_examples=DEFAULT_SETTINGS.max_examples if max_examples is None else max_examples,
    deadline=deadline,
    phases=DEFAULT_SETTINGS.phases if phases is None else tuple(phases),
  )

  def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
    func._propcheck_settings = configured
    return func

  return decorator


def _example_stream(strategies_by_name: Mapping[str, SearchStrategy[Any]], max_examples: int, seed: int) -> list[dict[str, Any]]:
  edge_values = {name: strategy.edge_cases() for name, strategy in strategies_by_name.items()}
  edge_examples = []
  max_edges = max((len(values) for values in edge_values.values()), default=0)
  for i in range(max_edges):
    example = {}
    for name, strategy in strategies_by_name.items():
      values = edge_values[name]
      if values:
        example[name] = _clone(values[i % len(values)])
      else:
        example[name] = strategy.generate(random.Random(seed + i))
    edge_examples.append(example)

  examples = edge_examples[:max_examples]
  remaining = max(0, max_examples - len(examples))
  for i in range(remaining):
    rng = random.Random(seed + 10_000 + i)
    examples.append({name: strategy.generate(rng) for name, strategy in strategies_by_name.items()})
  return examples


def _shrink_example(func: Callable[..., Any], args: tuple[Any, ...], call_kwargs: dict[str, Any],
                    strategies_by_name: Mapping[str, SearchStrategy[Any]], example: dict[str, Any]) -> dict[str, Any]:
  best = _clone(example)
  improved = True
  while improved:
    improved = False
    for name, strategy in strategies_by_name.items():
      current = best[name]
      for candidate in strategy.shrink(current):
        shrunk = _clone(best)
        shrunk[name] = candidate
        try:
          func(*args, **call_kwargs, **_clone(shrunk))
        except Exception:
          best = shrunk
          improved = True
          break
      if improved:
        break
  return best


def given(**strategies_by_name: SearchStrategy[Any]):
  if not strategies_by_name:
    raise ValueError("given() requires at least one named strategy")

  def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
    @functools.wraps(func)
    def wrapper(*args: Any, **call_kwargs: Any) -> None:
      configured = _get_settings(wrapper)
      examples = _example_stream(strategies_by_name, configured.max_examples, _DEFAULT_SEED)
      for example in examples:
        try:
          func(*args, **call_kwargs, **_clone(example))
        except Exception as exc:
          failing_example = example
          if Phase.shrink in configured.phases:
            failing_example = _shrink_example(func, args, call_kwargs, strategies_by_name, example)
          raise _coerce_exception(exc, failing_example) from exc

    wrapper._propcheck_settings = _get_settings(func)
    return wrapper

  return decorator


def integers(*, min_value: int | None = None, max_value: int | None = None) -> SearchStrategy[int]:
  return IntegerStrategy(min_value=min_value, max_value=max_value)


def sampled_from(values: Sequence[T]) -> SearchStrategy[T]:
  return SampledFromStrategy(values)


def booleans() -> SearchStrategy[bool]:
  return BooleanStrategy()


def binary(*, min_size: int = 0, max_size: int | None = None) -> SearchStrategy[bytes]:
  return BinaryStrategy(min_size=min_size, max_size=max_size)


def lists(element_strategy: SearchStrategy[T], *, min_size: int = 0, max_size: int | None = None) -> SearchStrategy[list[T]]:
  return ListStrategy(element_strategy, min_size=min_size, max_size=max_size)


def dictionaries(key_strategy: SearchStrategy[Any], value_strategy: SearchStrategy[Any], *,
                 min_size: int = 0, max_size: int | None = None) -> SearchStrategy[dict[Any, Any]]:
  return DictStrategy(key_strategy, value_strategy, min_size=min_size, max_size=max_size)


def fixed_dictionaries(mapping: Mapping[Any, SearchStrategy[Any]]) -> SearchStrategy[dict[Any, Any]]:
  return FixedDictionariesStrategy(mapping)


def builds(fn: Callable[..., T], *strategies: SearchStrategy[Any]) -> SearchStrategy[T]:
  return BuildsStrategy(fn, strategies)
