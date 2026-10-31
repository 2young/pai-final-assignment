"""World pose from Gazebo PosePublisher via TFMessage bridge (not odometry)."""
import math
import time


class PoseProvider:
    def __init__(self, node):
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import qos_profile_sensor_data
        self.pose = None
        self.received = 0
        self.stamp = None
        self.subscription = node.create_subscription(
            TFMessage, '/model/vehicle_blue/pose', self.callback, qos_profile_sensor_data)

    def callback(self, message):
        for transform in message.transforms:
            if transform.child_frame_id != 'vehicle_blue':
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            stamp = (transform.header.stamp.sec, transform.header.stamp.nanosec)
            if stamp == self.stamp:
                continue  # Paused simulation must become stale even if republished.
            self.stamp = stamp
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            pose = (t.x, t.y, yaw)
            if all(math.isfinite(v) for v in pose):
                self.pose, self.received = pose, time.monotonic()

    def current(self):
        if self.pose is None or time.monotonic()-self.received > 1.5:
            raise RuntimeError('World pose missing or stale')
        return self.pose
