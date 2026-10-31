"""Explicit live-scene integration test. Moves walls/robot and deletes a wall.

Requires the updated maze/bridges. Restart the maze after this test.
Run with ROS sourced: python3 /workspace/tests/gazebo_live_obstacles.py
"""
import ast
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'app'))
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from object_tracker import ObjectTracker
from pose_provider import PoseProvider
from grid_map import load_config
from contact_goal import ContactMonitor,footprint_free
from ros_feedback import spin_some


def service(name,kind,request):
    result=subprocess.run(['gz','service','-s',f'/world/maze/{name}',
                           '--reqtype',kind,'--reptype','gz.msgs.Boolean',
                           '--timeout','2000','--req',request],
                          capture_output=True,text=True,timeout=5,check=True)
    assert 'data: true' in result.stdout,result.stdout+result.stderr


def place(name,x,y,z,yaw=0):
    service('set_pose','gz.msgs.Pose',f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}')


def main():
    rclpy.init();node=Node('verify_live_obstacle_scene')
    cfg=load_config();tracker=ObjectTracker(node,cfg);robot=PoseProvider(node)
    contacts=ContactMonitor(node,'red_box');messages=[]
    sub=node.create_subscription(Twist,'/model/vehicle_blue/cmd_vel',
        lambda m:messages.append((time.monotonic(),m.linear.x,m.angular.z)),100)
    process=None;output=None
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:spin_some(node,timeout_sec=.01)
    def start():
        nonlocal process,output
        spin(.3);messages.clear();contacts.arm()
        output=tempfile.TemporaryFile(mode='w+t')
        process=subprocess.Popen(['python3','/workspace/app/robot_control.py',
            '赤いオブジェクトまで行ってください'],stdout=output,stderr=subprocess.STDOUT,text=True)
    def finish(reason=None,stationary=False,live=None):
        deadline=time.monotonic()+120
        while process.poll() is None and time.monotonic()<deadline:
            spin(.03)
            if live is not None:
                assert footprint_free(live,robot.current(),'red_box'),'Unsafe observed footprint'
        assert process.poll() is not None,'CLI timeout'
        spin(.3);output.seek(0);text=output.read();output.close()
        print(text,flush=True)
        assert messages and messages[-1][1:]==(0.,0.)
        if reason:
            assert process.returncode==1 and reason in text,text
            assert 'CONTACT_REACHED' not in text
        else:
            assert process.returncode==0 and 'CONTACT_REACHED' in text,text
            assert contacts.hit,'No independent chassis/box contact'
        if stationary:assert all(m[1:]==(0.,0.) for m in messages)
        before=robot.current();spin(.6)
        assert math.dist(before[:2],robot.current()[:2])<.01,'Robot did not stop'
        return text
    def wait_motion(contact=False):
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            spin(.03)
            if any(m[1]>=.25 if contact else m[1:]!=(0.,0.) for m in messages):return
            if process.poll() is not None:
                output.seek(0);raise AssertionError(output.read())
        raise AssertionError('No expected motion')
    def stopped_after(changed):
        zeros=[m[0] for m in messages if m[0]>=changed and m[1:]==(0.,0.)]
        assert zeros,'No zero Twist after scene change'
        return zeros[0]-changed
    try:
        # A long wall is moved and rotated; its former footprint becomes a
        # straight, traversable corridor. A second obstacle is also moved.
        place('vehicle_blue',-5,-4,.325)
        place('red_box',5,-4,.4)
        place('detour',0,-8.5,.5,math.pi/2)
        place('northeast_block',8,9,.5,math.pi/4)
        spin(.4);live=tracker.snapshot()
        assert len(tracker.providers)==len(cfg['walls'])+len(cfg['objects'])
        wall=next(o for o in live['walls'] if o['id']=='detour')
        assert wall['center']==[0,-8.5] and abs(wall['yaw']-math.pi/2)<.001
        assert all(math.isclose(v,e,abs_tol=1e-6) for v,e in zip(wall['size'][:2],[7.,.3]))
        start();text=finish(live=live)
        line=next(line for line in text.splitlines() if line.startswith('A*:'))
        route=ast.literal_eval(line.split('waypoints=',1)[1])
        assert len(route)==2 and all(abs(p[1]+4)<.01 for p in route),route
        assert text.count('Live obstacle:')==len(cfg['walls'])
        print('PASS: moved/rotated walls; real straight route through old wall; contact and zero Twist',flush=True)

        place('vehicle_blue',-5,-4,.325)
        start();wait_motion();changed=time.monotonic()
        place('west_wall',-8,1,.5)
        finish('west_wall: object moved during navigation')
        print(f'PASS: wall moved during A*: first zero after {stopped_after(changed):.3f}s',flush=True)

        place('vehicle_blue',2.25,-4,.325)
        start();wait_motion(contact=True);changed=time.monotonic()
        place('west_wall',-8,1,.5,.2)
        finish('west_wall: object moved during navigation')
        print(f'PASS: wall rotated during contact: first zero after {stopped_after(changed):.3f}s',flush=True)

        place('vehicle_blue',2.25,-4,.325)
        place('northwest_block',5,-4,.5)
        start();finish('red_box: overlaps northwest_block',stationary=True)
        print('PASS: target overlaps observed wall; all Twist zero',flush=True)
        place('northwest_block',-8,8,.5)

        place('vehicle_blue',2.25,-4,.325)
        start();wait_motion(contact=True);changed=time.monotonic()
        service('remove','gz.msgs.Entity','name: "northeast_block", type: MODEL')
        finish('northeast_block: contact object pose missing or stale')
        print(f'PASS: wall pose lost during contact: first zero after {stopped_after(changed):.3f}s',flush=True)
        start();finish('No fresh object world pose: [\'northeast_block\']',stationary=True)
        print('PASS: missing wall at command start; named failure; all Twist zero',flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
