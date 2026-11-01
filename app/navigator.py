"""Waypoint feedback with configurable speeds and clearance guards."""
import math
import time
from pose_provider import PoseProvider
from motion_config import MotionSettings
from route_metrics import MotionMetrics
from ros_feedback import spin_some
from follower_config import FollowerSettings


def steering(pose, goal, settings=MotionSettings()):
    x, y, yaw = pose
    distance = math.dist((x,y),goal)
    error = math.atan2(math.sin(math.atan2(goal[1]-y,goal[0]-x)-yaw),
                       math.cos(math.atan2(goal[1]-y,goal[0]-x)-yaw))
    angular = max(-settings.max_angular_speed,
                  min(settings.max_angular_speed, settings.heading_gain*error))
    linear = min(settings.max_linear_speed, settings.distance_gain*distance) if abs(error) < 0.10 else 0.0
    return float(linear), float(angular)


def stop(pub):
    from geometry_msgs.msg import Twist
    for _ in range(5):
        pub.publish(Twist())
        time.sleep(0.05)


class Navigator:
    def __init__(self, node, pub, grid, config=None):
        self.node, self.pub, self.grid = node, pub, grid
        self.provider = PoseProvider(node)
        self.settings = MotionSettings.from_config(config if config is not None else
                                                   (grid.config if grid is not None else {}))
        self.follower_settings=FollowerSettings.from_config(config if config is not None else
                                                    (grid.config if grid is not None else {}))

    def current(self):
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            spin_some(self.node, timeout_sec=0.1)
            if self.provider.pose is not None:
                return self.provider.current()
        raise RuntimeError('No world pose; start maze launcher and pose bridge')

    def follow(self, route, timeout=240, guard=None):
        if hasattr(route,'motions'):
            return self._follow_pose(route,timeout,guard)
        if getattr(self,'follower_settings',None) is not None and self.follower_settings.controller=='pure_pursuit':
            return self._follow_pursuit(route,timeout,guard)
        from geometry_msgs.msg import Twist
        deadline = time.monotonic()+timeout
        minimum_clearance = math.inf
        started = time.monotonic()
        metrics = MotionMetrics()
        try:
            for index, goal in enumerate(route[1:], 1):
                best, progress = math.inf, time.monotonic()
                while True:
                    spin_some(self.node, timeout_sec=0.05)
                    if guard is not None:
                        guard()
                    pose = self.provider.current()
                    metrics.observe_pose(pose)
                    minimum_clearance = min(minimum_clearance, self.grid.clearance(pose[:2]))
                    if not self.grid.point_free(pose[:2]):
                        raise RuntimeError('Robot outside safe map clearance')
                    distance = math.dist(pose[:2], goal)
                    tolerance = 0.025 if index < len(route)-1 else 0.10
                    if distance < tolerance:
                        stop(self.pub)
                        metrics.stopped()
                        break
                    if distance < best-0.02:
                        best, progress = distance, time.monotonic()
                    if time.monotonic() > deadline or time.monotonic()-progress > 25:
                        raise RuntimeError('Navigation timeout / stalled waypoint')
                    # Refuse tracking deviations that would cut across obstacles.
                    if not self.grid.segment_free(pose[:2], goal):
                        raise RuntimeError('Tracking deviation: next segment unsafe')
                    linear, angular = steering(pose, goal, self.settings)
                    metrics.observe_command(linear,angular)
                    msg = Twist()
                    msg.linear.x, msg.angular.z = linear, angular
                    self.pub.publish(msg)
                    time.sleep(0.03)
                print(f'Waypoint {index}: {goal}, error={distance:.3f}m', flush=True)
            # Process observations that arrived while the final STOP was sent.
            spin_some(self.node)
            if guard is not None:
                guard()
            final_pose = self.provider.current()
            metrics.observe_pose(final_pose)
            error = math.dist(final_pose[:2], route[-1])
            if error > 0.10 or not self.grid.point_free(final_pose[:2]):
                raise RuntimeError('Final stopped pose is outside goal tolerance or safe map')
            minimum_clearance = min(minimum_clearance, self.grid.clearance(final_pose[:2]))
            print(f'ARRIVED: pose={final_pose}, goal={route[-1]}, error={error:.3f}m, '
                  f'minimum_footprint_clearance={minimum_clearance:.3f}m, '
                  f'elapsed={time.monotonic()-started:.1f}s, '
                  f'rotation_episodes={metrics.episodes}, '
                  f'total_yaw={math.degrees(metrics.rotation):.1f}deg', flush=True)
            return metrics.summary()
        finally:
            stop(self.pub)

    def _follow_pose(self,route,timeout,guard):
        """Execute certified moving arcs, explicit pivots and gear changes."""
        from dataclasses import replace
        from geometry_msgs.msg import Twist
        from path_geometry import ArcPath
        from pure_pursuit import PursuitController,stopping_trajectory_free
        from route_metrics import simulation_time
        from pose_provider import angle_difference
        from pose_planner import PoseSettings
        scene=self.grid;started=time.monotonic();metrics=MotionMetrics()
        minimum=math.inf;maximum_error=0.;maximum_heading_error=0.;maximum_control_time=0.;sim_start=None
        blocks=route.blocks()
        print(f'FOLLOW_PATH: controller=pose_pursuit, blocks={len(blocks)}, '
              f'length={sum(abs(m.v)*m.duration for m in route.motions):.2f}m',flush=True)

        def observe():
            nonlocal minimum,sim_start
            spin_some(self.node,timeout_sec=.02)
            if guard is not None:guard()
            pose=self.provider.current();now=simulation_time(self.provider.stamp)
            if time.monotonic()-self.provider.received>self.follower_settings.pose_timeout:
                raise RuntimeError('Pose follower robot pose missing or stale')
            if time.monotonic()-started>timeout:raise RuntimeError('Pose navigation timeout')
            if not scene.free(pose):raise RuntimeError('Robot footprint outside safe clearance')
            minimum=min(minimum,scene.clearance(pose));metrics.observe_pose(pose,self.provider.stamp)
            if sim_start is None:sim_start=now
            return pose,now

        def publish(v,w):
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w)
            metrics.observe_command(v,w);self.pub.publish(msg);time.sleep(.02)

        def align(yaw):
            nonlocal maximum_control_time
            # Endpoint correction has the same braking/sweep guard as a planned
            # pivot; the old assumption of unrestricted rotation is not used.
            previous=None;v=w=0.;began=time.monotonic()
            while True:
                pose,now=observe()
                if previous is None:
                    previous=(pose,now);publish(0.,0.);continue
                dt=now-previous[1]
                if dt<0 or dt>self.follower_settings.max_feedback_interval+1e-9:
                    raise RuntimeError('Pose pivot feedback time moved backwards or interval too large')
                if dt==0:publish(v,w);continue
                actual_v=math.dist(pose[:2],previous[0][:2])/dt
                actual_w=angle_difference(pose[2],previous[0][2])/dt;previous=(pose,now)
                error=angle_difference(yaw,pose[2])
                if abs(error)<.04 and actual_v<.03 and abs(actual_w)<.05:break
                if time.monotonic()-began>30:raise RuntimeError('Pose pivot timeout')
                desired=(max(-self.settings.max_angular_speed,min(self.settings.max_angular_speed,1.8*error))
                         if actual_v<.03 else 0.)
                w+=max(-self.follower_settings.angular_acceleration*dt,
                       min(self.follower_settings.angular_acceleration*dt,desired-w))
                control_started=time.perf_counter()
                if not stopping_trajectory_free(scene,pose,0.,w,actual_v,actual_w,
                                                 self.follower_settings,collision=scene):
                    raise RuntimeError('No safe pose pivot braking trajectory')
                maximum_control_time=max(maximum_control_time,time.perf_counter()-control_started)
                publish(0.,w)
            stop(self.pub);metrics.stopped()

        try:
            for direction,poses in blocks:
                if direction==0:
                    align(poses[-1][2]);continue
                # Keep the primitive geometry intact. XY-only corner rounding
                # cannot certify this orientation-dependent corridor.
                path=ArcPath([p[:2] for p in poses])
                motion=(self.settings if direction>0 else replace(self.settings,
                    max_linear_speed=min(self.settings.max_linear_speed,PoseSettings.from_config(scene.config).reverse_speed)))
                controller=PursuitController(scene,path,motion,self.follower_settings,
                                             collision=scene,direction=direction,reference_poses=poses)
                best=0.;progress=time.monotonic()
                while True:
                    pose,now=observe()
                    virtual=(*pose[:2],pose[2]+(math.pi if direction<0 else 0.))
                    control_started=time.perf_counter()
                    v,w,done=controller.command(virtual,now)
                    maximum_control_time=max(maximum_control_time,time.perf_counter()-control_started)
                    maximum_error=max(maximum_error,controller.max_error)
                    maximum_heading_error=max(maximum_heading_error,controller.max_heading_error)
                    if done:break
                    if controller.progress>best+.02 or abs(w)>.02:
                        best=max(best,controller.progress);progress=time.monotonic()
                    if time.monotonic()-progress>25:raise RuntimeError('Pose follower stalled')
                    publish(direction*v,w)
                stop(self.pub);metrics.stopped()
                # A requested terminal heading is part of the route contract.
                align(poses[-1][2])
            final,now=observe();error=math.dist(final[:2],route[-1])
            yaw_error=abs(angle_difference(final[2],route.poses[-1][2]))
            if error>.10 or yaw_error>.08:raise RuntimeError('Final pose outside position / heading tolerance')
            print(f'ARRIVED: pose={final}, goal={route[-1]}, error={error:.3f}m, '
                  f'minimum_footprint_clearance={minimum:.3f}m, elapsed={time.monotonic()-started:.2f}s, '
                  f'sim_elapsed={now-sim_start:.2f}s, max_actual_speed={metrics.max_speed:.3f}m/s, '
                  f'max_cross_track={maximum_error:.3f}m, rotation_episodes={metrics.episodes}, '
                  f'max_heading_error={math.degrees(maximum_heading_error):.1f}deg, '
                  f'max_control_ms={maximum_control_time*1000:.2f}, '
                  f'total_yaw={math.degrees(metrics.rotation):.1f}deg',flush=True)
            return metrics.summary()
        finally:
            stop(self.pub)

    def _follow_pursuit(self,route,timeout,guard):
        from geometry_msgs.msg import Twist
        from path_geometry import rounded_path,ArcPath
        from pure_pursuit import PursuitController
        from route_metrics import simulation_time
        started=time.monotonic();metrics=MotionMetrics();minimum=math.inf
        sim_start=None;best=0.;progress_time=started
        try:
            points,stops,rounded=rounded_path(self.grid,route,self.follower_settings.smoothing_distance)
            path=ArcPath(points,stops)
            controller=PursuitController(self.grid,path,self.settings,self.follower_settings)
            print(f'FOLLOW_PATH: controller=pure_pursuit, length={path.length:.2f}m, '
                  f'rounded_corners={rounded}, stop_corners={len(stops)}',flush=True)
            while True:
                spin_some(self.node,timeout_sec=.02)
                if guard is not None:guard()
                pose=self.provider.current()
                if time.monotonic()-self.provider.received>self.follower_settings.pose_timeout:
                    raise RuntimeError('Pursuit robot pose missing or stale')
                stamp=self.provider.stamp;now=simulation_time(stamp)
                if sim_start is None:sim_start=now
                if not self.grid.point_free(pose[:2]):
                    raise RuntimeError('Robot outside safe map clearance')
                metrics.observe_pose(pose,stamp)
                minimum=min(minimum,self.grid.clearance(pose[:2]))
                v,w,done=controller.command(pose,now)
                if done:break
                # Rotation is meaningful progress while aligning at a corner.
                if controller.progress>best+.02 or abs(w)>.02:
                    best=max(best,controller.progress);progress_time=time.monotonic()
                if time.monotonic()-started>timeout or time.monotonic()-progress_time>25:
                    raise RuntimeError('Pursuit timeout / stalled')
                msg=Twist();msg.linear.x=v;msg.angular.z=w
                metrics.observe_command(v,w);self.pub.publish(msg);time.sleep(.02)
            stop(self.pub);spin_some(self.node)
            if guard is not None:guard()
            final=self.provider.current();error=math.dist(final[:2],route[-1])
            if error>.10 or not self.grid.point_free(final[:2]):
                raise RuntimeError('Final stopped pose is outside goal tolerance or safe map')
            print(f'ARRIVED: pose={final}, goal={route[-1]}, error={error:.3f}m, '
                  f'minimum_footprint_clearance={minimum:.3f}m, elapsed={time.monotonic()-started:.2f}s, '
                  f'sim_elapsed={simulation_time(self.provider.stamp)-sim_start:.2f}s, '
                  f'max_actual_speed={metrics.max_speed:.3f}m/s, max_cross_track={controller.max_error:.3f}m, '
                  f'rotation_episodes={metrics.episodes}, total_yaw={math.degrees(metrics.rotation):.1f}deg',flush=True)
            return metrics.summary()
        finally:
            stop(self.pub)
