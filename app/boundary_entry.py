"""Straight, orientation-preserving entry from a boundary into A* clearance."""
import math
import time
from contact_goal import footprint_free
from pose_provider import angle_difference
from navigator import stop
from contact_speed import FEEDBACK_WALL_TIMEOUT
from ros_feedback import spin_some


def center_clear(config,point):
    p=config['robot']['radius']+config['robot']['margin']+.10+math.sqrt(2)*config['resolution']
    x,y=point;x0,y0,x1,y1=config['bounds']
    if not (x0+p<x<x1-p and y0+p<y<y1-p):return False
    return not any(abs(x-o['center'][0])<=o['size'][0]/2+p and
                   abs(y-o['center'][1])<=o['size'][1]/2+p
                   for o in config['walls']+config['objects'])


def plan_boundary_entry(config,pose):
    x,y,yaw=pose;x0,y0,x1,y1=config['bounds']
    p=config['robot']['radius']+config['robot']['margin']+.10+math.sqrt(2)*config['resolution']
    if x0+p<x<x1-p and y0+p<y<y1-p:
        return None
    if not footprint_free(config,pose,None):
        raise ValueError('Robot footprint violates operating area / obstacle clearance; reposition robot')
    choices=[]
    for sign in (-1,1):
        for step in range(1,121):
            distance=step*.05
            point=(x+sign*math.cos(yaw)*distance,y+sign*math.sin(yaw)*distance)
            if not footprint_free(config,(*point,yaw),None):break
            if center_clear(config,point):
                choices.append({'goal':point,'heading':yaw,'sign':sign,'start':(x,y)})
                break
    if not choices:
        raise ValueError('No safe straight entry from boundary; reposition robot/orientation')
    return min(choices,key=lambda entry:math.dist(entry['start'],entry['goal']))


def execute_boundary_entry(navigator,config,entry,guard):
    from geometry_msgs.msg import Twist
    heading=entry['heading'];cx,cy=entry['goal'];sx,sy=entry['start'];sign=entry['sign']
    started=progress=time.monotonic();best=math.inf
    print(f'BOUNDARY_ENTRY: goal={entry["goal"]}, direction={"reverse" if sign<0 else "forward"}',flush=True)
    try:
        spin_some(navigator.node)
        while True:
            spin_some(navigator.node,timeout_sec=.02);guard()
            x,y,yaw=navigator.provider.current()
            if time.monotonic()-navigator.provider.received>FEEDBACK_WALL_TIMEOUT:
                raise RuntimeError('Boundary entry pose missing or stale')
            if not footprint_free(config,(x,y,yaw),None):
                raise RuntimeError('Boundary entry clearance lost')
            error=angle_difference(heading,yaw)
            lateral=abs(-(x-sx)*math.sin(heading)+(y-sy)*math.cos(heading))
            if abs(error)>.10 or lateral>.05:
                raise RuntimeError('Boundary entry tracking deviation')
            remaining=sign*((cx-x)*math.cos(heading)+(cy-y)*math.sin(heading))
            if remaining<=.02:break
            if remaining<best-.01:best,progress=remaining,time.monotonic()
            if time.monotonic()-started>45 or time.monotonic()-progress>3:
                raise RuntimeError('Boundary entry timeout / stalled')
            msg=Twist()
            msg.linear.x=float(sign*min(.2,navigator.settings.max_linear_speed,.8*remaining))
            msg.angular.z=float(max(-.15,min(.15,1.8*error)))
            navigator.pub.publish(msg);time.sleep(.02)
        stop(navigator.pub)
        spin_some(navigator.node)
        guard()
        pose=navigator.provider.current()
        if math.dist(pose[:2],entry['goal'])>.05:
            raise RuntimeError('Boundary entry final position error')
        print(f'BOUNDARY_ENTRY_DONE: pose={pose}',flush=True)
    finally:
        stop(navigator.pub)
