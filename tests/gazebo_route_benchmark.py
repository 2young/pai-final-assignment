"""Explicit real-Gazebo CLI benchmark; reset maze before each comparison run."""
import argparse
import copy
import math
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'app'))
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from contact_goal import ContactMonitor
from grid_map import GridMap,load_config
from pose_provider import PoseProvider,angle_difference


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--label',default='run',help='Log label only; does not select a planner')
    args=parser.parse_args()
    rclpy.init();node=Node('route_benchmark')
    pose=PoseProvider(node)
    contact=ContactMonitor(node,'red_box')
    commands=[];episodes=0;rotating=False
    def command(msg):
        nonlocal episodes,rotating
        now=abs(msg.linear.x)<1e-6 and abs(msg.angular.z)>.02
        if now and not rotating:
            episodes+=1
        rotating=now
        commands.append((msg.linear.x,msg.angular.z))
    subscription=node.create_subscription(Twist,'/model/vehicle_blue/cmd_vel',command,100)
    cfg=copy.deepcopy(load_config());cfg['objects']=[]
    walls=GridMap(cfg)
    process=None;last_stamp=None;last_yaw=None
    total_angle=0.;min_clearance=math.inf
    try:
        started=time.monotonic()
        process=subprocess.Popen(['python3','/workspace/app/robot_control.py',
                                  '赤いオブジェクトまで行ってください'],
                                 stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        announced=False
        while process.poll() is None and time.monotonic()-started<240:
            rclpy.spin_once(node,timeout_sec=.02)
            if commands and any(abs(v)>0 for v in commands[-1]) and not announced:
                print(f'BENCHMARK_RUNNING: {args.label}',flush=True);announced=True
            if pose.pose is not None and pose.stamp!=last_stamp:
                state=pose.current()
                if last_yaw is not None:
                    total_angle+=abs(angle_difference(state[2],last_yaw))
                last_yaw=state[2];last_stamp=pose.stamp
                min_clearance=min(min_clearance,walls.clearance(state[:2]))
        output=process.communicate(timeout=2)[0]
        elapsed=time.monotonic()-started
        end=time.monotonic()+.3
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
        print(output,flush=True)
        assert process.returncode==0 and 'CONTACT_REACHED' in output
        assert contact.hit and commands and commands[-1]==(0.,0.)
        print(f'BENCHMARK: label={args.label}, elapsed={elapsed:.3f}s, '
              f'rotation_episodes={episodes}, total_yaw={math.degrees(total_angle):.1f}deg, '
              f'min_wall_circle_clearance={min_clearance:.3f}m, '
              f'final_twist={commands[-1]}, contacts={contact.details}',flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
