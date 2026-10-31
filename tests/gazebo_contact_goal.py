"""Explicit integration check: moves robot through the real contact-goal CLI.

Requires a running updated maze and a robot at a reachable stand-off position.
Run with ROS sourced: python3 /workspace/tests/gazebo_contact_goal.py
"""
import math
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'app'))
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from contact_goal import ContactMonitor,footprint_free
from grid_map import load_config
from object_tracker import ObjectTracker
from pose_provider import PoseProvider


def main():
    rclpy.init()
    node=Node('verify_real_contact_goal')
    monitor=ContactMonitor(node,'red_box')
    provider=PoseProvider(node)
    tracker=ObjectTracker(node,load_config())
    commands=[]
    subscription=node.create_subscription(
        Twist,'/model/vehicle_blue/cmd_vel',lambda m:commands.append((m.linear.x,m.angular.z)),100)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
    process=None
    try:
        spin(.5)
        live=tracker.snapshot()
        monitor.arm()
        process=subprocess.Popen(['python3','/workspace/app/robot_control.py',
                                  '--goal-mode','contact',
                                  '赤いオブジェクトまで行ってください'],
                                 stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        deadline=time.monotonic()+180
        while process.poll() is None and time.monotonic()<deadline:
            spin(.05)
        output=process.communicate(timeout=2)[0]
        print(output,flush=True)
        spin(.5)
        assert process.returncode==0 and 'CONTACT_REACHED' in output
        assert monitor.hit, 'No independent robot/box contact observed'
        assert commands and commands[-1]==(0.,0.)
        before=provider.current()
        assert footprint_free(live,before,'red_box'),'Stopped robot outside operating area/clearance'
        spin(1)
        after=provider.current()
        drift=math.dist(before[:2],after[:2])
        assert drift<.01,drift
        print(f'PASS: contacts={monitor.details}, final Twist={commands[-1]}, '
              f'stationary_drift={drift:.6f}m',flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
