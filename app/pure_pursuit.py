"""Regulated pursuit with braking-trajectory checks on the inflated grid."""
import math
import bisect
from pose_provider import angle_difference
from footprint import integrate
from motion_config import DRIVE_ACCELERATION,DRIVE_ANGULAR_ACCELERATION


def curvature(pose,point):
    dx,dy=point[0]-pose[0],point[1]-pose[1]
    squared=dx*dx+dy*dy
    if squared<1e-10:return 0.
    local_y=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
    return 2*local_y/squared


def speed_cap(motion,settings,kappa,clearance,remaining,actual_speed):
    bend=abs(kappa)
    return max(0.,min(motion.max_linear_speed,
        math.sqrt(settings.lateral_acceleration/max(bend,1e-9)),
        motion.max_angular_speed/max(bend,1e-9),
        .45+1.5*max(0.,clearance),
        math.sqrt(2*settings.deceleration*max(0.,remaining-.04-settings.reaction_time*actual_speed))))


def preview_speed_cap(path,progress,limit,actual_speed,motion,settings):
    """Brake before an upcoming bend, rather than after entering its curvature."""
    cap=motion.max_linear_speed
    for station,kappa in path.curvatures:
        if station<progress-.05 or station>limit:continue
        corner_speed=min(math.sqrt(settings.lateral_acceleration/kappa),motion.max_angular_speed/kappa)
        distance=max(0.,station-progress-settings.reaction_time*actual_speed-.08)
        cap=min(cap,math.sqrt(corner_speed**2+2*settings.deceleration*distance))
    return cap


def stopping_trajectory_free(grid,pose,v,w,actual_v,actual_w,settings,collision=None,direction=1):
    """Predict command latency and a braking tail; check every swept chord.

    The robot circle includes all orientations and grid cells are conservatively
    inflated. Actual feedback is included instead of assuming an instant turn.
    """
    current=pose;speed=max(0.,actual_v);omega=actual_w
    duration=settings.reaction_time+max(max(speed,v)/settings.deceleration,
                                      max(abs(actual_w),abs(w))/settings.angular_acceleration)+.1
    dt=.025;elapsed=0.
    while elapsed<duration:
        if elapsed<settings.reaction_time:
            target=v;target_w=w
        else:
            target=max(0.,v-settings.deceleration*(elapsed-settings.reaction_time))
            target_w=(w*target/v if v>1e-6 else 0.)
        speed+=max(-settings.deceleration*dt,min(settings.acceleration*dt,target-speed))
        omega+=max(-settings.angular_acceleration*dt,min(settings.angular_acceleration*dt,target_w-omega))
        nxt=integrate(current,speed,omega,dt)
        if collision is None:
            if not grid.segment_free(current[:2],nxt[:2]):return False
        else:
            physical=(*current[:2],current[2]+(math.pi if direction<0 else 0.))
            # Cover the discretization of actuator acceleration in addition
            # to the continuous sweep inside this integration interval.
            pad=.5*(DRIVE_ACCELERATION+collision.radius*DRIVE_ANGULAR_ACCELERATION)*dt*dt
            if not collision.motion_free(physical,direction*speed,omega,dt,extra=pad):return False
        current=nxt;elapsed+=dt
    return True


class PursuitController:
    def __init__(self,grid,path,motion,settings,collision=None,direction=1,reference_poses=None):
        self.grid,self.path,self.motion,self.settings=grid,path,motion,settings
        self.progress=0.;self.stop_index=0;self.last_time=None;self.last_pose=None
        self.v=self.w=0.;self.max_error=0.;self.actual_v=self.actual_w=0.
        self.aligning=True
        self.collision=collision;self.direction=direction
        self.max_heading_error=0.;self.reference_yaws=None
        if reference_poses is not None:
            self.reference_yaws=[];last=None
            for pose in reference_poses:
                if last is None or math.dist(last[:2],pose[:2])>1e-8:
                    self.reference_yaws.append(pose[2]);last=pose
            if len(self.reference_yaws)!=len(path.points):raise ValueError('Pose reference/path mismatch')

    def _safe(self,pose,v,w):
        return stopping_trajectory_free(self.grid,pose,v,w,self.actual_v,self.actual_w,
                                        self.settings,self.collision,self.direction)

    def command(self,pose,sim_time):
        if not all(math.isfinite(v) for v in (*pose,sim_time)):
            raise RuntimeError('Invalid pursuit feedback')
        if self.last_time is None:
            self.last_time=sim_time;self.last_pose=pose;return 0.,0.,False
        dt=sim_time-self.last_time
        if dt<0 or dt>self.settings.max_feedback_interval+1e-9:
            raise RuntimeError('Pursuit feedback time moved backwards or interval too large')
        if dt==0:return self.v,self.w,False
        self.actual_v=math.dist(pose[:2],self.last_pose[:2])/dt
        self.actual_w=angle_difference(pose[2],self.last_pose[2])/dt
        self.last_time=sim_time;self.last_pose=pose
        limit=(self.path.stops[self.stop_index] if self.stop_index<len(self.path.stops) else self.path.length)
        self.progress,error=self.path.project(pose[:2],self.progress,limit)
        self.max_error=max(error,self.max_error)
        if error>self.settings.max_cross_track:
            raise RuntimeError('Pursuit cross-track error exceeded limit')
        if self.reference_yaws is not None:
            i=min(len(self.path.points)-1,max(0,bisect.bisect_right(self.path.stations,self.progress)-1))
            expected=self.reference_yaws[i]+(math.pi if self.direction<0 else 0.)
            deviation=abs(angle_difference(pose[2],expected))
            self.max_heading_error=max(self.max_heading_error,deviation)
            if deviation>self.settings.max_heading_error:
                raise RuntimeError('Pose tracking heading error exceeded limit')
        remaining=max(0.,limit-self.progress)
        end_distance=math.dist(pose[:2],self.path.points[-1])
        if limit==self.path.length and end_distance<.08:
            self.v=self.w=0.;return 0.,0.,True
        if remaining<.06 and limit<self.path.length:
            target=self.path.point(min(self.path.length,limit+.3))
            heading=math.atan2(target[1]-pose[1],target[0]-pose[0])
            error_yaw=angle_difference(heading,pose[2])
            if abs(error_yaw)<.08 and self.actual_v<.05:
                self.stop_index+=1
                self.v=self.w=0.
                return 0.,0.,False
            return self._rotate(pose,error_yaw,dt)
        if self.aligning:
            target=self.path.point(min(limit,self.progress+.2))
            error_yaw=angle_difference(math.atan2(target[1]-pose[1],target[0]-pose[0]),pose[2])
            if abs(error_yaw)<.10 and self.actual_v<.05:
                self.aligning=False;self.v=self.w=0.
            else:
                return self._rotate(pose,error_yaw,dt)
        lookahead=max(self.settings.lookahead_min,min(self.settings.lookahead_max,
                        self.settings.lookahead_min+self.settings.lookahead_time*self.actual_v))
        for factor in (1.,.7,.45):
            point=self.path.point(min(limit,self.progress+max(.2,lookahead*factor)))
            if self.collision is None and not self.grid.segment_free(pose[:2],point):continue
            heading=math.atan2(point[1]-pose[1],point[0]-pose[0])
            error_yaw=angle_difference(heading,pose[2])
            if abs(error_yaw)>self.settings.rotate_threshold:
                self.aligning=True
                return self._rotate(pose,error_yaw,dt)
            kappa=curvature(pose,point)
            margin=self.grid.config['robot']['margin']
            # Pose-aware control checks the full braking sweep below. Lateral
            # clearance alone need not cap speed on a straight narrow corridor.
            clearance=(math.inf if self.collision is not None else self.grid.clearance(pose[:2])-margin)
            cap=speed_cap(self.motion,self.settings,kappa,clearance,remaining,self.actual_v)
            cap=min(cap,preview_speed_cap(self.path,self.progress,limit,self.actual_v,self.motion,self.settings))
            for factor_speed in (1.,.75,.5,.25,0.):
                requested=cap*factor_speed
                v=max(self.v-self.settings.deceleration*dt,
                      min(requested,self.v+self.settings.acceleration*dt))
                w=max(-self.motion.max_angular_speed,min(self.motion.max_angular_speed,v*kappa))
                w=max(self.w-self.settings.angular_acceleration*dt,
                      min(w,self.w+self.settings.angular_acceleration*dt))
                if self._safe(pose,v,w):
                    self.v,self.w=float(v),float(w)
                    return self.v,self.w,False
        # Brake first. Pose-aware pivots additionally certify the full rotation.
        target=self.path.point(min(limit,self.progress+.25))
        self.aligning=True
        return self._rotate(pose,angle_difference(math.atan2(target[1]-pose[1],target[0]-pose[0]),pose[2]),dt)

    def _rotate(self,pose,error,dt):
        v=max(0.,self.v-self.settings.deceleration*dt)
        if self.collision is not None and max(self.actual_v,v)<.05:
            physical=(*pose[:2],pose[2]+(math.pi if self.direction<0 else 0.))
            if not self.collision.motion_free(physical,0.,math.copysign(1.,error),abs(error)):
                raise RuntimeError('Unplanned pivot sweep unsafe; issue a new navigation command')
        desired=(max(-self.motion.max_angular_speed,min(self.motion.max_angular_speed,1.8*error))
                 if max(self.actual_v,v)<.05 else 0.)
        w=max(self.w-self.settings.angular_acceleration*dt,min(desired,self.w+self.settings.angular_acceleration*dt))
        if not self._safe(pose,v,w):
            raise RuntimeError('No safe pursuit braking trajectory')
        self.v,self.w=float(v),float(w)
        return self.v,self.w,False
