from __future__ import annotations

from collections.abc import Sequence
from typing import TypeVar

Number = float | int
T = TypeVar("T")

# Drop-in pure-Python replacements for common numpy scalar helpers.
# These are often faster for our cases than numpy.


def clip(x: Number, lo: Number, hi: Number) -> Number:
  if x < lo:
    return lo
  if x > hi:
    return hi
  return x


def sign(x: Number) -> int:
  """Match np.sign: -1, 0, or 1."""
  return (x > 0) - (x < 0)


def interp(x: float, xp: Sequence[float], fp: Sequence[float]) -> float:
  """1D linear interpolation matching numpy.interp (edge hold for extrapolation)."""
  n = len(xp)
  if n != len(fp):
    raise ValueError("xp and fp must have the same length")
  if n == 0:
    raise ValueError("xp and fp must be non-empty")
  if n == 1:
    return float(fp[0])

  if x <= xp[0]:
    return float(fp[0])
  if x >= xp[-1]:
    return float(fp[-1])

  # Binary search for rightmost xp[i] <= x
  lo = 0
  hi = n - 1
  while hi - lo > 1:
    mid = (lo + hi) // 2
    if xp[mid] > x:
      hi = mid
    else:
      lo = mid

  x0, x1 = xp[lo], xp[hi]
  y0, y1 = fp[lo], fp[hi]
  if x1 == x0:
    return float(y0)
  return float(y0 + (x - x0) * (y1 - y0) / (x1 - x0))


def arange(start: float, stop: float | None = None, step: float = 1.0) -> list[float]:
  """Like numpy.arange, returning a Python list."""
  if stop is None:
    stop = start
    start = 0.0
  if step == 0:
    raise ValueError("step must not be zero")

  # Integer-count generation to limit float accumulation drift (numpy-like)
  result: list[float] = []
  n = 0
  if step > 0:
    while True:
      val = start + n * step
      if val >= stop:
        break
      result.append(val)
      n += 1
  else:
    while True:
      val = start + n * step
      if val <= stop:
        break
      result.append(val)
      n += 1
  return result


def linspace(start: float, stop: float, num: int = 50) -> list[float]:
  """Like numpy.linspace (endpoint included), returning a Python list."""
  if num < 0:
    raise ValueError("number of samples must be non-negative")
  if num == 0:
    return []
  if num == 1:
    return [float(start)]
  step = (stop - start) / (num - 1)
  return [start + i * step for i in range(num)]


def concatenate(seqs: Sequence[Sequence[T]]) -> list[T]:
  out: list[T] = []
  for s in seqs:
    out.extend(s)
  return out


def isclose(a: float, b: float, rtol: float = 1e-05, atol: float = 1e-08) -> bool:
  """Match numpy.isclose for scalars."""
  return abs(a - b) <= (atol + rtol * abs(b))
