"""Indexable as (speed_bp, angle_v) for lateral.apply_std_steer_angle_limits."""
from dataclasses import dataclass


@dataclass
class AngleRateLimit:
  speed_bp: list[float]
  angle_v: list[float]

  def __getitem__(self, i):
    return (self.speed_bp, self.angle_v)[i]
