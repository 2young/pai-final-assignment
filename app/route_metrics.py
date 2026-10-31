"""Measured yaw travel and episodes of commanded in-place rotation."""
import math
import time
from collections import deque
from pose_provider import angle_difference


def simulation_time(stamp):
    if stamp is None:
        raise RuntimeError('No simulation timestamp for contact feedback')
    return stamp[0]+stamp[1]*1e-9


class MotionMetrics:
    def __init__(self):
        self.started=time.monotonic()
        self.episodes=0
        self.rotation=0.
        self.last_yaw=None
        self.rotating=False
        self.last_speed=None
        self.max_speed=0.
        self.speed_samples=deque(maxlen=150)
        self.speed_pose=None
        self.speed_time=None

    def observe_pose(self,pose,stamp=None):
        if self.last_yaw is not None:
            self.rotation+=abs(angle_difference(pose[2],self.last_yaw))
        self.last_yaw=pose[2]
        if stamp is not None:
            now=simulation_time(stamp)
            if self.speed_time is not None:
                dt=now-self.speed_time
                if dt<0:
                    raise RuntimeError('Simulation time moved backwards during contact')
                if dt==0:
                    return
                self.last_speed=math.dist(pose[:2],self.speed_pose)/dt
                self.max_speed=max(self.max_speed,self.last_speed)
                self.speed_samples.append((now,self.last_speed))
            self.speed_pose=pose[:2]
            self.speed_time=now

    def speed_before(self,stamp):
        if stamp is None:
            return None
        now=simulation_time(stamp)
        samples=[speed for end,speed in self.speed_samples if now-.12<=end<=now]
        return max(samples) if samples else None

    def observe_command(self,linear,angular):
        rotating=abs(linear)<1e-6 and abs(angular)>.02
        if rotating and not self.rotating:
            self.episodes+=1
        self.rotating=rotating

    def stopped(self):
        self.rotating=False

    def summary(self):
        return {'elapsed':time.monotonic()-self.started,'rotation_episodes':self.episodes,
                'total_yaw_degrees':math.degrees(self.rotation),'max_actual_speed':self.max_speed}
