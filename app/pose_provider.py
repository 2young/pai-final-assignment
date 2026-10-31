"""Fresh world poses from Gazebo PosePublisher via a TFMessage bridge."""
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class WorldPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    @property
    def planar(self):
        return self.x, self.y, self.yaw


def angle_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


class PoseProvider:
    def __init__(self, node, model_name='vehicle_blue'):
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.model_name = model_name
        self.pose = self.state = None
        self.received = 0
        self.stamp = None
        self.error = 'World pose missing'
        self.subscription = node.create_subscription(
            TFMessage, f'/model/{model_name}/pose', self.callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def callback(self, message):
        for transform in message.transforms:
            if transform.child_frame_id != self.model_name:
                continue
            if transform.header.frame_id != 'maze':
                self.pose = self.state = None
                self.error = 'Expected maze world frame'
                continue
            stamp = (transform.header.stamp.sec, transform.header.stamp.nanosec)
            if stamp == self.stamp:
                continue  # Duplicate timestamps cannot keep a paused world fresh.
            self.stamp = stamp
            t, q = transform.transform.translation, transform.transform.rotation
            values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
            norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
            if not all(math.isfinite(v) for v in values) or abs(norm-1) > 0.01:
                self.pose = self.state = None
                self.error = 'Invalid world pose/quaternion'
                continue
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.state = WorldPose(t.x, t.y, t.z, roll, pitch, yaw)
            self.pose, self.received = self.state.planar, time.monotonic()

    def current_state(self):
        if self.state is None:
            raise RuntimeError(f'{self.model_name}: {self.error}')
        if time.monotonic()-self.received > 1.5:
            raise RuntimeError(f'{self.model_name}: World pose missing or stale')
        return self.state

    def current(self):
        return self.current_state().planar
