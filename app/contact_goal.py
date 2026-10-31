"""Plan a stand-off route, then creep until the target's contact sensor fires."""
import math
import subprocess
import time
from path_planner import plan
from pose_provider import angle_difference
from navigator import stop

CONTACT_SPEED = 0.10


def footprint_free(config, pose, target_id):
    """Orientation-aware conservative rectangle, excluding only the touch target."""
    x,y,yaw = pose
    if not all(math.isfinite(v) for v in pose):
        return False
    left,bottom,right,top = config['robot']['footprint_bounds']
    margin = config['robot']['margin']
    c,s = math.cos(yaw),math.sin(yaw)
    corners = [(x+c*a-s*b,y+s*a+c*b)
               for a,b in [(left-margin,bottom-margin),(right+margin,bottom-margin),
                           (right+margin,top+margin),(left-margin,top+margin)]]
    x0,y0,x1,y1 = config['bounds']
    if any(not (x0 < px < x1 and y0 < py < y1) for px,py in corners):
        return False
    for obj in config['walls']+config['objects']:
        if obj['id'] == target_id:
            continue
        ox,oy = obj['center']
        sx,sy = obj['size'][:2]
        other = [(ox+a*sx/2,oy+b*sy/2) for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]]
        separated = False
        for ax,ay in [(1,0),(0,1),(c,s),(-s,c)]:
            p = [px*ax+py*ay for px,py in corners]
            q = [px*ax+py*ay for px,py in other]
            if max(p) < min(q) or max(q) < min(p):
                separated = True
                break
        if not separated:
            return False
    return True


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


def plan_contact(grid, start, target):
    x,y = target['center']
    sx,sy = target['size'][:2]
    choices = []
    # Nearby stand-off offsets avoid rejecting a feasible side solely because
    # its nominal goal falls in a conservatively occupied boundary grid cell.
    for dx,dy,half in [(-1,0,sx/2),(1,0,sx/2),(0,-1,sy/2),(0,1,sy/2)]:
        for extra in (.35,.25,.45):
            distance = half+grid.padding+extra
            approach = (x+dx*distance,y+dy*distance)
            heading = math.atan2(-dy,-dx)
            front = grid.config['robot']['footprint_bounds'][2]
            radius = target_ray_radius(target,heading+math.pi)
            travel = distance-radius-front+.08
            n = max(1,math.ceil(travel/.02))
            if not all(footprint_free(grid.config,
                       (approach[0]+math.cos(heading)*travel*i/n,
                        approach[1]+math.sin(heading)*travel*i/n,heading),target['id'])
                       for i in range(n+1)):
                continue
            try:
                route = plan(grid,start,approach)
                choices.append(route)
                break
            except ValueError:
                continue
    if not choices:
        raise ValueError('No reachable approach point with safe contact corridor')
    return min(choices,key=lambda r:sum(math.dist(a,b) for a,b in zip(r,r[1:])))


class ContactMonitor:
    def __init__(self,node,target_id):
        from ros_gz_interfaces.msg import Contacts
        from rclpy.qos import qos_profile_sensor_data
        self.target_id = target_id
        self.hit = False
        self.details = None
        self.min_stamp = None
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
                self.hit = True
                self.details = (a,b)

    def arm(self,min_stamp=None):
        self.hit = False
        self.details = None
        self.min_stamp = min_stamp


def touch(navigator, config, target, monitor, guard, timeout=60):
    import rclpy
    from geometry_msgs.msg import Twist
    cx,cy = target['center']
    started = time.monotonic()
    best,progress = math.inf,started
    aligned = False
    monitor.arm(getattr(navigator.provider,'stamp',None))
    print(f'CONTACT approach: target={target["id"]}, speed={CONTACT_SPEED}m/s',flush=True)
    try:
        while True:
            rclpy.spin_once(navigator.node,timeout_sec=.02)
            guard()
            pose = navigator.provider.current()
            if not footprint_free(config,pose,target['id']):
                raise RuntimeError('Contact approach would violate wall/boundary clearance')
            if monitor.hit:
                stop(navigator.pub)
                print(f'CONTACT_REACHED: target={target["id"]}, pose={pose}, '
                      f'collisions={monitor.details}',flush=True)
                return
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
                else:
                    # Rotation occurs only at the collision-free stand-off point.
                    msg=Twist()
                    msg.angular.z=float(max(-.5,min(.5,1.8*error)))
                    navigator.pub.publish(msg)
                    time.sleep(.02)
                    continue
            if distance < best-.01:
                best,progress=distance,time.monotonic()
            if time.monotonic()-progress>3:
                raise RuntimeError('Contact approach stalled without sensor confirmation')
            msg=Twist()
            msg.linear.x=CONTACT_SPEED
            msg.angular.z=float(max(-.15,min(.15,1.8*error)))
            navigator.pub.publish(msg)
            time.sleep(.02)
    finally:
        stop(navigator.pub)
