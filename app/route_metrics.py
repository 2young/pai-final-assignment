"""Measured yaw travel and episodes of commanded in-place rotation."""
import math
import time
from pose_provider import angle_difference


class MotionMetrics:
    def __init__(self):
        self.started=time.monotonic()
        self.episodes=0
        self.rotation=0.
        self.last_yaw=None
        self.rotating=False

    def observe_pose(self,pose):
        if self.last_yaw is not None:
            self.rotation+=abs(angle_difference(pose[2],self.last_yaw))
        self.last_yaw=pose[2]

    def observe_command(self,linear,angular):
        rotating=abs(linear)<1e-6 and abs(angular)>.02
        if rotating and not self.rotating:
            self.episodes+=1
        self.rotating=rotating

    def stopped(self):
        self.rotating=False

    def summary(self):
        return {'elapsed':time.monotonic()-self.started,'rotation_episodes':self.episodes,
                'total_yaw_degrees':math.degrees(self.rotation)}
