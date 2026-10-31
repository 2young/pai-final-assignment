"""Explicit final-approach benchmark using real CLI, simulation-stamped poses.

Place the robot at a stand-off point, facing the box, before each comparison.
This script drives the robot to contact; it does not reset the scene.
"""
import argparse
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
from pose_provider import PoseProvider


class ContactObserver(ContactMonitor):
    def __init__(self,node):
        self.event_stamp=self.event_wall=None
        super().__init__(node,'red_box')

    def callback(self,message):
        old=self.hit
        super().callback(message)
        if self.hit and not old:
            self.event_stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            self.event_wall=time.monotonic()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--label',default='current',help='Result label, not mode selection')
    parser.add_argument('--max-touch-speed',type=float,default=.13)
    args=parser.parse_args()
    rclpy.init();node=Node('contact_speed_benchmark')
    provider=PoseProvider(node);contact=ContactObserver(node)
    commands=[];first_forward=None
    def command(msg):
        nonlocal first_forward
        commands.append((msg.linear.x,msg.angular.z))
        if msg.linear.x>1e-6 and first_forward is None:
            first_forward=time.monotonic()
            print(f'CONTACT_SPEED_BENCHMARK_RUNNING: {args.label}',flush=True)
    subscription=node.create_subscription(Twist,'/model/vehicle_blue/cmd_vel',command,100)
    poses=[];last_stamp=None
    def spin(seconds):
        nonlocal last_stamp
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if provider.pose is not None and provider.stamp!=last_stamp:
                t=provider.stamp[0]+provider.stamp[1]*1e-9
                poses.append((t,provider.current()[:2]));last_stamp=provider.stamp
    process=None
    try:
        spin(.3)
        started=time.monotonic()
        process=subprocess.Popen(['python3','/workspace/app/robot_control.py',
                                  '--goal-mode','contact','赤いオブジェクトまで行ってください'],
                                 stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        deadline=time.monotonic()+150
        while process.poll() is None and time.monotonic()<deadline:
            spin(.03)
        output=process.communicate(timeout=2)[0]
        elapsed=time.monotonic()-started
        print(output,flush=True);spin(.4)
        assert process.returncode==0 and 'CONTACT_REACHED' in output
        assert contact.hit and first_forward is not None
        assert commands and commands[-1]==(0.,0.)
        speeds=[(b[0],math.dist(a[1],b[1])/(b[0]-a[0]))
                for a,b in zip(poses,poses[1:]) if b[0]>a[0]]
        near=[v for t,v in speeds if contact.event_stamp-.12<=t<=contact.event_stamp]
        assert near,'No pre-contact speed observations'
        contact_speed=max(near)
        assert contact_speed<=args.max_touch_speed,contact_speed
        contact_pose=[p for t,p in poses if t<=contact.event_stamp][-1]
        stopped=provider.current()[:2]
        displacement=math.dist(contact_pose,stopped)
        assert displacement<.05,displacement
        before=provider.current();spin(1);after=provider.current()
        drift=math.dist(before[:2],after[:2]);assert drift<.01,drift
        after_speeds=[(t,v) for t,v in speeds if t>=contact.event_stamp]
        stable=[after_speeds[i][0] for i in range(len(after_speeds)-2)
                if all(v<.01 for _,v in after_speeds[i:i+3])]
        assert stable,'No stationary speed interval after contact'
        print(f'CONTACT_SPEED_BENCHMARK: label={args.label}, cli_elapsed={elapsed:.3f}s, '
              f'forward_to_contact={contact.event_wall-first_forward:.3f}s, '
              f'peak_actual_speed={max(v for t,v in speeds):.3f}m/s, '
              f'pre_contact_actual_speed={contact_speed:.3f}m/s, '
              f'estimated_stop_delay={max(0,stable[0]-contact.event_stamp):.3f}s, '
              f'contact_to_stop_displacement={displacement:.4f}m, '
              f'stationary_drift={drift:.6f}m, final_twist={commands[-1]}',flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
