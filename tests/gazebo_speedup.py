"""Repeated real-CLI benchmarks; changes robot/box poses. Requires updated maze.

ROS sourced: python3 tests/gazebo_speedup.py --label before --runs 3
Use --app-root to run an archived implementation against the same world.
"""
import argparse
import ast
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--label',default='current')
    parser.add_argument('--runs',type=int,default=3)
    parser.add_argument('--cases',nargs='+',default=['straight','corners','maze','narrow','no_route'])
    parser.add_argument('--output',default='/tmp/speedup')
    parser.add_argument('--app-root',default='/workspace')
    parser.add_argument('--controller',choices=['waypoint','pure_pursuit'])
    args=parser.parse_args();root=Path(args.app_root)
    sys.path.insert(0,str(root/'app'))
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from grid_map import GridMap,load_config
    from object_tracker import ObjectTracker
    from pose_provider import PoseProvider,angle_difference
    from contact_goal import ContactMonitor,footprint_free
    from ros_feedback import spin_some
    from route_metrics import simulation_time
    courses={
        'straight':((-5,4,0),(5,4)),
        'open_straight':((-4.5,5,0),(5,5)),
        'corners':((-5,-4,0),(5,-1.95)),
        'maze':((-5,-4,0),(5,4)),
        'narrow':((-5,-4,math.pi/2),(-5,3)),
        'no_route':((-5,-4,0),(-5,-2))}
    destination=Path(args.output);destination.mkdir(parents=True,exist_ok=True)
    cfg=load_config();(destination/'experiment_config.json').write_text(json.dumps(
        {'label':args.label,'runs':args.runs,'cases':courses,'controller_override':args.controller,'config':cfg},ensure_ascii=False,indent=2))
    rclpy.init();node=Node('speedup_benchmark');tracker=ObjectTracker(node,cfg)
    provider=PoseProvider(node);monitor=ContactMonitor(node,'red_box');commands=[]
    subscription=node.create_subscription(Twist,'/model/vehicle_blue/cmd_vel',
        lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)),100)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:spin_some(node,timeout_sec=.01)
    def place(name,x,y,z,yaw=0):
        req=f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
        req+=f'orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
        result=subprocess.run(['gz','service','-s','/world/maze/set_pose','--reqtype','gz.msgs.Pose',
            '--reptype','gz.msgs.Boolean','--timeout','2000','--req',req],capture_output=True,text=True,check=True)
        assert 'data: true' in result.stdout,result.stdout+result.stderr
    rows=[];process=None
    try:
        for case in args.cases:
            start,target=courses[case]
            for repetition in range(1,(1 if case=='no_route' else args.runs)+1):
                place('red_box',*target,.4);place('vehicle_blue',*start[:2],.325,start[2]);spin(.5)
                live=None if case=='no_route' else tracker.snapshot()
                wall_cfg=json.loads(json.dumps(cfg));wall_cfg['objects']=[]
                walls=GridMap(wall_cfg)
                monitor.arm();commands.clear();samples=[];previous=None
                distance=0.;max_speed=0.;min_clearance=math.inf;stopped=0.;yaw=0.;rotations=0;rotating=False
                sim_start=simulation_time(provider.stamp);began=time.monotonic()
                log_path=destination/f'{args.label}-{case}-{repetition}.log'
                with log_path.open('w') as log:
                    command=['python3',str(root/'app/robot_control.py'),'赤いオブジェクトまで行ってください']
                    if args.controller:command.extend(['--controller',args.controller])
                    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,text=True)
                    while process.poll() is None and time.monotonic()-began<200:
                        spin(.02)
                        state=provider.current();now=simulation_time(provider.stamp)
                        if previous is not None and now>previous[0]:
                            dt=now-previous[0];step=math.dist(state[:2],previous[1][:2]);speed=step/dt
                            distance+=step;max_speed=max(max_speed,speed)
                            yaw+=abs(angle_difference(state[2],previous[1][2]))
                            if speed<.02:stopped+=dt
                            samples.append((now,*state,speed,*(commands[-1][1:] if commands else (0.,0.))))
                        if live is not None:
                            assert footprint_free(live,state,'red_box'),f'{case}: unsafe robot footprint: {state}'
                        min_clearance=min(min_clearance,walls.clearance(state[:2]))
                        previous=(now,state)
                    assert process.poll() is not None,'Benchmark command timeout'
                spin(.3);output=log_path.read_text();sim_end=simulation_time(provider.stamp)
                rotating=False
                for _,v,w in commands:
                    active=abs(v)<1e-6 and abs(w)>.02
                    rotations+=int(active and not rotating);rotating=active
                expected=1 if case=='no_route' else 0
                success=process.returncode==expected and (case=='no_route' or ('CONTACT_REACHED' in output and monitor.hit))
                assert commands and commands[-1][1:]==(0.,0.),'Missing final zero Twist'
                if case=='no_route':assert all(m[1:]==(0.,0.) for m in commands)
                first=provider.current();spin(.6);assert math.dist(first[:2],provider.current()[:2])<.01
                line=next((line for line in output.splitlines() if line.startswith('A*:')),None)
                route=ast.literal_eval(line.split('waypoints=',1)[1]) if line else []
                row=dict(label=args.label,case=case,run=repetition,success=success,
                    wall_seconds=time.monotonic()-began,sim_seconds=sim_end-sim_start,
                    distance=distance,max_speed=max_speed,stopped_sim_seconds=stopped,
                    rotation_episodes=rotations,total_yaw_degrees=math.degrees(yaw),
                    min_wall_circle_clearance=min_clearance,waypoints=len(route),
                    planned_length=sum(math.dist(a,b) for a,b in zip(route,route[1:])))
                rows.append(row)
                with (destination/'metrics.csv').open('w') as stream:
                    writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader();writer.writerows(rows)
                with (destination/f'{args.label}-{case}-{repetition}-trace.csv').open('w') as stream:
                    writer=csv.writer(stream);writer.writerow(['sim_time','x','y','yaw','actual_speed','cmd_linear','cmd_angular']);writer.writerows(samples)
                print(json.dumps(row),flush=True)
                if not success:print(output,flush=True);raise AssertionError(f'{case} failed')
    finally:
        if process is not None and process.poll() is None:process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
