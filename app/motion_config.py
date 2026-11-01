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


# Explicitly applied to the generated DiffDrive, also bounds contact ramps.
DRIVE_ACCELERATION = 1.0
DRIVE_ANGULAR_ACCELERATION = 2.0


@dataclass(frozen=True)
class ContactSettings:
    max_speed: float = 0.40
    touch_speed: float = 0.10
    acceleration: float = 0.30
    deceleration: float = 0.35
    safety_buffer: float = 0.20
    reaction_time: float = 0.20

    def __post_init__(self):
        for field in fields(self):
            value=getattr(self,field.name)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError(f'contact_approach.{field.name} must be finite and positive')
        if self.touch_speed>self.max_speed:
            raise ValueError('contact_approach.touch_speed must not exceed max_speed')
        if max(self.acceleration,self.deceleration)>DRIVE_ACCELERATION:
            raise ValueError('Contact acceleration/deceleration exceeds vehicle acceleration limit')

    @classmethod
    def from_config(cls,config):
        values=config.get('contact_approach',{})
        unknown=set(values)-{field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f'Unknown contact_approach settings: {sorted(unknown)}')
        return cls(**values)
