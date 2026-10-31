"""Shared speed limits for the controller and generated Gazebo vehicle."""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class MotionSettings:
    max_linear_speed: float = 0.7
    max_angular_speed: float = 0.5
    distance_gain: float = 0.8
    heading_gain: float = 1.8

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'navigation.{field.name} must be finite and positive')

    @classmethod
    def from_config(cls, config):
        values = config.get('navigation', {})
        unknown = set(values)-{field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f'Unknown navigation settings: {sorted(unknown)}')
        return cls(**values)
