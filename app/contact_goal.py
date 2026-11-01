"""Plan a stand-off route, then creep until the target's contact sensor fires."""
import math
import subprocess
import time
from path_planner import plan,route_score,selection_score
from target_resolver import approach_candidates
from pose_provider import angle_difference
from navigator import stop
from route_metrics import MotionMetrics,simulation_time
from motion_config import ContactSettings,MotionSettings
from contact_speed import projected_gap,ApproachSpeedController,FEEDBACK_WALL_TIMEOUT
from ros_feedback import spin_some
from footprint import footprint_free


def target_ray_radius(target, outward_yaw):
    """Distance from oriented-box center to its surface on this centerline."""
    angle = outward_yaw-target.get('yaw',0)
    sx,sy = target.get('body_size', target['size'])[:2]
    return min(half/abs(component) if abs(component)>1e-10 else math.inf
               for half,component in [(sx/2,math.cos(angle)),(sy/2,math.sin(angle))])


def can_resume_touch(config, pose, target):
    """Resume a nearly aligned final approach after a previous contact stop."""
    x,y,yaw = pose
    cx,cy = target['center']
    heading = math.atan2(cy-y,cx-x)
    if abs(angle_difference(heading,yaw)) >= .03:
        return False
    distance = math.dist((x,y),(cx,cy))
    radius = target_ray_radius(target,heading+math.pi)
    front = config['robot']['footprint_bounds'][2]
    padding = config['robot']['radius']+config['robot']['margin']
    if not radius+front-.08 <= distance <= radius+padding+.45:
        return False
    if not footprint_free(config,pose,target['id']):
        return False
    travel = max(0,distance-radius-front+.08)
    n = max(1,math.ceil(travel/.02))
    return all(footprint_free(config,(x+math.cos(heading)*travel*i/n,
                                      y+math.sin(heading)*travel*i/n,heading),target['id'])
               for i in range(n+1))


def plan_contact(grid, start, target, start_yaw=None):
    x,y = target['center']
    choices = []
    # Nearby stand-off offsets avoid rejecting a feasible side solely because
    # its nominal goal falls in a conservatively occupied boundary grid cell.
    accepted=set()
    for approach,heading in approach_candidates(grid,target,start,start_yaw,extras=(.35,.25,.45,.15)):
        grid.check_budget()
        key=round(heading,8)
        if key in accepted:
            continue
        distance=math.dist(approach,(x,y))
        front = grid.config['robot']['footprint_bounds'][2]
        radius = target_ray_radius(target,heading+math.pi)
        # Validate up to the expected physical touch (front bound has 1cm
        # allowance). Runtime footprint checks still guard any overtravel.
        travel = distance-radius-front+.01
        n = max(1,math.ceil(travel/.02))
        if not all(footprint_free(grid.config,
                   (approach[0]+math.cos(heading)*travel*i/n,
                    approach[1]+math.sin(heading)*travel*i/n,heading),target['id'])
                   for i in range(n+1)):
            continue
        try:
            route = plan(grid,start,approach,start_yaw=start_yaw,final_yaw=heading)
            choices.append((route,heading))
            accepted.add(key)
        except ValueError:
            continue
    if not choices:
        raise ValueError('No reachable approach point with safe contact corridor')
    return min(choices,key=lambda item:selection_score(grid,item[0],start_yaw,item[1]))[0]


class ContactMonitor:
    def __init__(self,node,target_id):
        from ros_gz_interfaces.msg import Contacts
        from rclpy.qos import qos_profile_sensor_data
        self.target_id = target_id
        self.hit = False
        self.details = None
        self.min_stamp = None
        self.contact_stamp = None
        self.subscription = node.create_subscription(
            Contacts,f'/model/{target_id}/contact',self.callback,qos_profile_sensor_data)

    def require_sensor(self):
        # Contact sensors publish only on contact, so an empty-message heartbeat
        # cannot be used. Verify the Gazebo publisher before any motion instead.
        try:
            result = subprocess.run(['gz','topic','-i','-t',f'/model/{self.target_id}/contact'],
                                    capture_output=True,text=True,timeout=5,check=True)
        except (subprocess.SubprocessError, OSError) as exc:
            raise RuntimeError('Cannot verify contact sensor; restart maze') from exc
        publishers = result.stdout.split('Subscribers')[0]
        if 'gz.msgs.Contacts' not in publishers:
            raise RuntimeError('No Gazebo contact sensor publisher; restart maze')

    def callback(self,message):
        if self.min_stamp is not None:
            stamp = (message.header.stamp.sec,message.header.stamp.nanosec)
            if stamp <= self.min_stamp:
                return
        for contact in message.contacts:
            a,b = contact.collision1.name,contact.collision2.name
            names = [a.split('::'),b.split('::')]
            if ((self.target_id in names[0] and 'vehicle_blue' in names[1]) or
                    (self.target_id in names[1] and 'vehicle_blue' in names[0])):
                if not self.hit and hasattr(message,'header'):
                    self.contact_stamp=(message.header.stamp.sec,message.header.stamp.nanosec)
                self.hit = True
                self.details = (a,b)

    def arm(self,min_stamp=None):
        self.hit = False
        self.details = None
        self.min_stamp = min_stamp
        self.contact_stamp = None


def touch(navigator, config, target, monitor, guard, timeout=60):
    from geometry_msgs.msg import Twist
    cx,cy = target['center']
    started = time.monotonic()
    best,progress = math.inf,started
    aligned = False
    monitor.arm(getattr(navigator.provider,'stamp',None))
    metrics=MotionMetrics()
    settings=ContactSettings.from_config(config)
    controller=ApproachSpeedController(settings,MotionSettings.from_config(config).max_linear_speed)
    print(f'CONTACT approach: target={target["id"]}, max_speed={controller.max_speed}m/s, '
          f'touch_speed={controller.touch_speed}m/s',flush=True)
    try:
        # Drain updates received during the previous waypoint's STOP sequence.
        spin_some(navigator.node)
        while True:
            spin_some(navigator.node,timeout_sec=.02)
            guard()
            pose = navigator.provider.current()
            stamp=getattr(navigator.provider,'stamp',None)
            if time.monotonic()-navigator.provider.received>FEEDBACK_WALL_TIMEOUT:
                raise RuntimeError('Contact robot pose missing or stale')
            metrics.observe_pose(pose,stamp)
            if not footprint_free(config,pose,target['id']):
                raise RuntimeError('Contact approach would violate wall/boundary clearance')
            if monitor.hit:
                stop(navigator.pub)
                speed=metrics.speed_before(getattr(monitor,'contact_stamp',None))
                measured='unknown' if speed is None else f'{speed:.3f}m/s'
                print(f'CONTACT_REACHED: target={target["id"]}, pose={pose}, '
                      f'collisions={monitor.details}, rotation_episodes={metrics.episodes}, '
                      f'total_yaw={math.degrees(metrics.rotation):.1f}deg, '
                      f'elapsed={time.monotonic()-started:.2f}s, '
                      f'max_actual_speed={metrics.max_speed:.3f}m/s, '
                      f'pre_contact_actual_speed={measured}',flush=True)
                return metrics.summary()
            x,y,yaw = pose
            distance = math.dist((x,y),(cx,cy))
            heading = math.atan2(cy-y,cx-x)
            error = angle_difference(heading,yaw)
            if time.monotonic()-started > timeout:
                raise RuntimeError('Contact approach timeout; no confirmed contact')
            front = config['robot']['footprint_bounds'][2]
            lower = target_ray_radius(target,heading+math.pi)+front-.08
            if distance < lower:
                raise RuntimeError('Contact travel limit reached without sensor confirmation')
            if aligned and abs(error) > .15:
                raise RuntimeError('Contact approach heading deviation')
            if not aligned:
                if abs(error)<.03:
                    aligned=True
                    best,progress=distance,time.monotonic()
                    controller.reset(simulation_time(stamp))
                else:
                    # Rotation occurs only at the collision-free stand-off point.
                    msg=Twist()
                    msg.angular.z=float(max(-.5,min(.5,1.8*error)))
                    metrics.observe_command(0.,msg.angular.z)
                    navigator.pub.publish(msg)
                    time.sleep(.02)
                    continue
            if distance < best-.01:
                best,progress=distance,time.monotonic()
            if time.monotonic()-progress>3:
                raise RuntimeError('Contact approach stalled without sensor confirmation')
            msg=Twist()
            msg.linear.x=controller.step(projected_gap(config,pose,target),
                                         metrics.last_speed or 0.,simulation_time(stamp))
            msg.angular.z=float(max(-.15,min(.15,1.8*error))) if msg.linear.x>0 else 0.
            metrics.observe_command(msg.linear.x,msg.angular.z)
            navigator.pub.publish(msg)
            time.sleep(.02)
    finally:
        stop(navigator.pub)
