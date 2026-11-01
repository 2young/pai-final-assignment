"""Explicit real-CLI pose planner verification; repositions all scene models.

Requires run_maze.sh. Saves live poses, commands and CLI logs; restores the
registered initial poses on exit. Not collected by pytest.
"""
import argparse
import copy
import gzip
import json
import math
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'app'))
import rclpy
from geometry_msgs.msg import Twist
from grid_map import load_config
from pose_provider import PoseProvider
from contact_goal import ContactMonitor
from route_metrics import simulation_time
from ros_feedback import spin_some


def independent_clear(config,pose):
    """Independent corner-projection SAT against observed wall AABBs."""
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    l,b,r,t=config['robot']['footprint_bounds'];m=config['robot']['margin']
    corners=[(x+c*u-s*v,y+s*u+c*v) for u,v in ((l-m,b-m),(r+m,b-m),(r+m,t+m),(l-m,t+m))]
    x0,y0,x1,y1=config['bounds']
    if any(not (x0<px<x1 and y0<py<y1) for px,py in corners):return False
    for obj in config['walls']:
        ox,oy=obj['center'];sx,sy=obj['size'][:2]
        other=[(ox+u*sx/2,oy+v*sy/2) for u,v in ((-1,-1),(1,-1),(1,1),(-1,1))]
        if not any(max(px*ax+py*ay for px,py in corners)<min(px*ax+py*ay for px,py in other) or
                   max(px*ax+py*ay for px,py in other)<min(px*ax+py*ay for px,py in corners)
                   for ax,ay in ((1,0),(0,1),(c,s),(-s,c))):return False
    return True


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--planner',choices=['auto','circle','hybrid'],default='auto')
    parser.add_argument('--runs',type=int,default=3)
    parser.add_argument('--cases',nargs='+',default=['open_straight','maze','reported','reverse'])
    parser.add_argument('--output',default='/tmp/pose-navigation')
    args=parser.parse_args();destination=Path(args.output);destination.mkdir(parents=True,exist_ok=True)
    cfg=load_config();rclpy.init();node=rclpy.create_node('pose_navigation_benchmark')
    provider=PoseProvider(node);monitor=ContactMonitor(node,'red_box');commands=[]
    subscription=node.create_subscription(Twist,'/model/vehicle_blue/cmd_vel',
        lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)),100)
    process=None;rows=[]

    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:spin_some(node,timeout_sec=.01)

    def place(name,x,y,z,yaw=0):
        request=f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
        last=''
        for attempt in range(3):
            try:
                result=subprocess.run(['gz','service','-s','/world/maze/set_pose','--reqtype','gz.msgs.Pose',
                    '--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],
                    capture_output=True,text=True,timeout=7)
                if result.returncode==0 and 'data: true' in result.stdout:return
                last=result.stdout+result.stderr
            except subprocess.TimeoutExpired as exc:last=str(exc)
            print(f'Retry idempotent set_pose: {name}, attempt={attempt+1}',flush=True)
        raise RuntimeError(last)

    def scene(case):
        config=copy.deepcopy(cfg);goal=None
        if case=='open_straight':start=(-4.5,5,0);config['objects'][0]['center']=[5,5]
        elif case=='maze':start=(-5,-4,0)
        elif case=='reported':
            start=(-7.771098613739145,-3.4186110496520996,3.1415900000000754)
            overrides={'dead_end':[-2,2.7871241569519043],'west_wall':[4.379763603210449,0],
                       'southeast_block':[3.735650062561035,-8.035699844360352]}
            for obj in config['walls']:obj['center']=overrides.get(obj['id'],obj['center'])
            config['objects'][0]['center']=[2.4746155738830566,-3.4161500930786133]
        elif case=='reverse':
            start=(0,1,math.pi/2);goal=(0,-1)
            for i,obj in enumerate(config['walls']):obj['center']=[40+i*10,0]
            config['walls'][0]['center']=[-1.8,0]
            config['walls'][3]['center']=[1.8,0]
            config['objects'][0]['center']=[8,8]
        else:raise ValueError(case)
        return config,start,goal

    def restore():
        for obj in cfg['walls']+cfg['objects']:place(obj['id'],*obj['center'],obj['size'][2]/2)
        place('vehicle_blue',*cfg['robot']['spawn'][:2],.325,cfg['robot']['spawn'][2])

    try:
        for case in args.cases:
            config,start,goal=scene(case)
            for repetition in range(1,args.runs+1):
                for obj in config['walls']+config['objects']:place(obj['id'],*obj['center'],obj['size'][2]/2)
                place('vehicle_blue',*start[:2],.325,start[2]);spin(.5)
                monitor.arm(provider.stamp);commands.clear();samples=[];previous=None;max_speed=0.;distance=0.
                prefix=f'{args.planner}-{case}-{repetition}';began=time.monotonic()
                sim_start=simulation_time(provider.stamp)
                (destination/f'{prefix}-scene.json').write_text(json.dumps({'config':config,'start':start,'goal':goal},indent=2))
                with (destination/f'{prefix}.log').open('w') as output:
                    command=['python3','/workspace/app/robot_control.py','--planner',args.planner]
                    command+=['--goal',*map(str,goal)] if goal is not None else ['赤いオブジェクトまで行ってください']
                    process=subprocess.Popen(command,stdout=output,stderr=subprocess.STDOUT,text=True)
                    while process.poll() is None and time.monotonic()-began<200:
                        spin(.02);pose=provider.current();now=simulation_time(provider.stamp)
                        assert independent_clear(config,pose),f'Unsafe footprint: {pose}'
                        if previous is not None and now>previous[0]:
                            dt=now-previous[0];step=math.dist(pose[:2],previous[1][:2])
                            speed=step/dt;distance+=step;max_speed=max(max_speed,speed)
                            samples.append((now,*pose,speed,*(commands[-1][1:] if commands else (0.,0.))))
                        previous=now,pose
                    assert process.poll() is not None,'Command timeout'
                spin(.3);text=(destination/f'{prefix}.log').read_text()
                success=process.returncode==0 and ('ARRIVED:' in text if goal is not None else 'CONTACT_REACHED' in text and monitor.hit)
                assert commands and commands[-1][1:]==(0.,0.),'No final zero Twist'
                final=provider.current();spin(.6)
                assert math.dist(final[:2],provider.current()[:2])<.01,'Vehicle did not remain stopped'
                row={'case':case,'planner':args.planner,'run':repetition,'success':success,
                     'wall_seconds':time.monotonic()-began,'sim_seconds':previous[0]-sim_start,
                     'distance':distance,'max_actual_speed':max_speed,'final_pose':final,
                     'reverse_observed':any(v<-.02 for _,v,w in commands)}
                rows.append(row)
                with gzip.open(destination/f'{prefix}-samples.json.gz','wt') as f:json.dump({'poses':samples,'commands':commands},f)
                (destination/'summary.json').write_text(json.dumps(rows,indent=2))
                print(json.dumps(row),flush=True)
                assert success,text
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=5)
        restore();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
