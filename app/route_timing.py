"""Comparable acceleration/curvature-aware estimates for candidate selection."""
import math
from path_geometry import ArcPath,rounded_path
from pose_provider import angle_difference
from motion_config import MotionSettings
from follower_config import FollowerSettings


def moving_time(points,direction,motion,settings,reverse_speed):
    path=ArcPath(points)
    if path.length<1e-8:return 0.
    count=max(2,math.ceil(path.length/.1))
    stations=[path.length*i/count for i in range(count+1)]
    limit=motion.max_linear_speed if direction>0 else min(motion.max_linear_speed,reverse_speed)
    speeds=[limit]*(count+1)
    for station,kappa in path.curvatures:
        index=min(count,max(0,round(station/path.length*count)))
        cap=min(limit,math.sqrt(settings.lateral_acceleration/kappa),motion.max_angular_speed/kappa)
        speeds[index]=min(speeds[index],cap)
    speeds[0]=speeds[-1]=0.
    for i in range(1,count+1):
        speeds[i]=min(speeds[i],math.sqrt(speeds[i-1]**2+2*settings.acceleration*(stations[i]-stations[i-1])))
    for i in range(count-1,-1,-1):
        speeds[i]=min(speeds[i],math.sqrt(speeds[i+1]**2+2*settings.deceleration*(stations[i+1]-stations[i])))
    return sum(2*(b-a)/max(1e-8,u+v) for a,b,u,v in zip(stations,stations[1:],speeds,speeds[1:]))


def estimate_time(grid,route,start_yaw,final_yaw=None):
    from pose_planner import PoseSettings
    motion=MotionSettings.from_config(grid.config);settings=FollowerSettings.from_config(grid.config)
    reverse=PoseSettings.from_config(grid.config).reverse_speed
    total=0.;yaw=start_yaw

    def rotation(destination):
        nonlocal yaw,total
        delta=abs(angle_difference(destination,yaw));yaw=destination
        if delta<.04:return
        # Symmetric triangular/trapezoidal angular acceleration profile.
        threshold=motion.max_angular_speed**2/settings.angular_acceleration
        total+=(2*math.sqrt(delta/settings.angular_acceleration) if delta<threshold else
                delta/motion.max_angular_speed+motion.max_angular_speed/settings.angular_acceleration)
        total+=.25  # Explicit STOP feedback settling.

    if hasattr(route,'motions'):
        for direction,poses in route.blocks():
            if direction==0:rotation(poses[-1][2]);continue
            rotation(poses[0][2])
            total+=moving_time([p[:2] for p in poses],direction,motion,settings,reverse)+.25
            yaw=poses[-1][2]
    else:
        points,stops,_=rounded_path(grid,route,settings.smoothing_distance)
        indices=[0,*stops,len(points)-1]
        for begin,end in zip(indices,indices[1:]):
            block=points[begin:end+1]
            if len(block)<2:continue
            rotation(math.atan2(block[1][1]-block[0][1],block[1][0]-block[0][0]))
            total+=moving_time(block,1,motion,settings,reverse)+.25
            yaw=math.atan2(block[-1][1]-block[-2][1],block[-1][0]-block[-2][0])
    if final_yaw is not None:rotation(final_yaw)
    return total
