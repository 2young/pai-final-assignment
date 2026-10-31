"""Conservative separation and simulation-time ramp for the final approach."""
import math

MAX_FEEDBACK_INTERVAL = 0.20
FEEDBACK_WALL_TIMEOUT = 0.30


def projected_gap(config,pose,target):
    """Lower bound on surface separation along the direction toward the box.

    Supporting planes enclose both footprints; using the original rotated box
    dimensions and the full robot front/side avoids a late slowdown at corners.
    """
    x,y,yaw=pose
    cx,cy=target['center']
    heading=math.atan2(cy-y,cx-x)
    box_angle=heading-target.get('yaw',0)
    sx,sy=target.get('body_size',target['size'])[:2]
    box_support=abs(math.cos(box_angle))*sx/2+abs(math.sin(box_angle))*sy/2
    angle=heading-yaw
    ux,uy=math.cos(angle),math.sin(angle)
    x0,y0,x1,y1=config['robot']['footprint_bounds']
    robot_support=max(x0*ux,x1*ux)+max(y0*uy,y1*uy)
    return math.dist((x,y),(cx,cy))-box_support-robot_support


class ApproachSpeedController:
    def __init__(self,settings,vehicle_max_speed):
        if not math.isfinite(vehicle_max_speed) or vehicle_max_speed<=0:
            raise ValueError('Invalid vehicle speed limit')
        self.settings=settings
        self.max_speed=min(settings.max_speed,vehicle_max_speed)
        self.touch_speed=min(settings.touch_speed,self.max_speed)
        self.speed=0.
        self.last_time=None

    def reset(self,sim_time):
        if not math.isfinite(sim_time):
            raise RuntimeError('Invalid contact feedback time')
        self.speed=0.
        self.last_time=sim_time

    def step(self,gap,actual_speed,sim_time):
        if not all(math.isfinite(v) for v in (gap,actual_speed,sim_time)) or actual_speed<0:
            raise RuntimeError('Invalid contact speed feedback')
        if self.last_time is None:
            self.reset(sim_time)
            return 0.
        dt=sim_time-self.last_time
        if dt<0:
            raise RuntimeError('Simulation time moved backwards during contact')
        if dt>MAX_FEEDBACK_INTERVAL+1e-9:
            raise RuntimeError('Contact feedback interval too large')
        if dt==0:
            return self.speed
        self.last_time=sim_time
        reserve=self.settings.safety_buffer+self.settings.reaction_time*max(actual_speed,self.speed)
        braking_gap=max(0.,gap-reserve)
        cap=min(self.max_speed,math.sqrt(self.touch_speed**2+2*self.settings.deceleration*braking_gap))
        if cap>=self.speed:
            self.speed=min(cap,self.speed+self.settings.acceleration*dt)
        else:
            self.speed=max(cap,self.speed-self.settings.deceleration*dt)
        return float(self.speed)
