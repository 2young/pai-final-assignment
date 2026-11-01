"""Validated settings for continuous path tracking, separate from drive limits."""
from dataclasses import dataclass,fields
import math
from motion_config import DRIVE_ACCELERATION,DRIVE_ANGULAR_ACCELERATION


@dataclass(frozen=True)
class FollowerSettings:
    controller: str = 'pure_pursuit'
    path_objective: str = 'travel_time'
    lookahead_min: float = .35
    lookahead_max: float = 1.0
    lookahead_time: float = .8
    tracking_margin: float = .10
    smoothing_distance: float = .9
    rotate_threshold: float = 1.4
    lateral_acceleration: float = .45
    acceleration: float = .5
    deceleration: float = .7
    angular_acceleration: float = 1.5
    reaction_time: float = .35
    max_cross_track: float = .35
    pose_timeout: float = .3
    max_feedback_interval: float = .2

    def __post_init__(self):
        if self.controller not in ('waypoint','pure_pursuit'):
            raise ValueError('path_following.controller must be waypoint or pure_pursuit')
        if self.path_objective not in ('turns','travel_time'):
            raise ValueError('path_following.path_objective must be turns or travel_time')
        for field in fields(self):
            if field.name in ('controller','path_objective'):continue
            value=getattr(self,field.name)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError(f'path_following.{field.name} must be finite and positive')
        if self.lookahead_min>self.lookahead_max:
            raise ValueError('lookahead_min must not exceed lookahead_max')
        if max(self.acceleration,self.deceleration)>DRIVE_ACCELERATION:
            raise ValueError('Path acceleration exceeds vehicle limit')
        if self.angular_acceleration>DRIVE_ANGULAR_ACCELERATION:
            raise ValueError('Path angular acceleration exceeds vehicle limit')
        if self.tracking_margin<.1:
            raise ValueError('Path tracking margin must be at least 0.1m')
        if self.reaction_time<self.pose_timeout+.04:
            raise ValueError('Reaction time must cover pose timeout and control latency')
        if self.rotate_threshold>math.pi/2:
            raise ValueError('rotate_threshold must not exceed pi/2')

    @classmethod
    def from_config(cls,config):
        values=config.get('path_following',{})
        if set(values)-{f.name for f in fields(cls)}:
            raise ValueError('Unknown path_following setting')
        return cls(**values)
