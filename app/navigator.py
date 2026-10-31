"""Low-speed waypoint feedback with timeout, stall and clearance guards."""
import math
import time
from pose_provider import PoseProvider


def steering(pose, goal):
    x, y, yaw = pose
    distance = math.dist((x,y),goal)
    error = math.atan2(math.sin(math.atan2(goal[1]-y,goal[0]-x)-yaw),
                       math.cos(math.atan2(goal[1]-y,goal[0]-x)-yaw))
    angular = max(-0.5,min(0.5,1.8*error))
    linear = min(0.35, 0.8*distance) if abs(error) < 0.10 else 0.0
    return linear, angular


def stop(pub):
    from geometry_msgs.msg import Twist
    for _ in range(5):
        pub.publish(Twist())
        time.sleep(0.05)


class Navigator:
    def __init__(self, node, pub, grid):
        self.node, self.pub, self.grid = node, pub, grid
        self.provider = PoseProvider(node)

    def current(self):
        import rclpy
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.provider.pose is not None:
                return self.provider.current()
        raise RuntimeError('No world pose; start maze launcher and pose bridge')

    def follow(self, route, timeout=240):
        import rclpy
        from geometry_msgs.msg import Twist
        deadline = time.monotonic()+timeout
        minimum_clearance = math.inf
        try:
            for index, goal in enumerate(route[1:], 1):
                best, progress = math.inf, time.monotonic()
                while True:
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                    pose = self.provider.current()
                    minimum_clearance = min(minimum_clearance, self.grid.clearance(pose[:2]))
                    if not self.grid.point_free(pose[:2]):
                        raise RuntimeError('Robot outside safe map clearance')
                    distance = math.dist(pose[:2], goal)
                    tolerance = 0.025 if index < len(route)-1 else 0.10
                    if distance < tolerance:
                        stop(self.pub)
                        break
                    if distance < best-0.02:
                        best, progress = distance, time.monotonic()
                    if time.monotonic() > deadline or time.monotonic()-progress > 25:
                        raise RuntimeError('Navigation timeout / stalled waypoint')
                    # Refuse tracking deviations that would cut across obstacles.
                    if not self.grid.segment_free(pose[:2], goal):
                        raise RuntimeError('Tracking deviation: next segment unsafe')
                    linear, angular = steering(pose, goal)
                    msg = Twist()
                    msg.linear.x, msg.angular.z = linear, angular
                    self.pub.publish(msg)
                    time.sleep(0.03)
                print(f'Waypoint {index}: {goal}, error={distance:.3f}m', flush=True)
            print(f'ARRIVED: pose={self.provider.current()}, goal={route[-1]}, '
                  f'minimum_footprint_clearance={minimum_clearance:.3f}m', flush=True)
        finally:
            stop(self.pub)
