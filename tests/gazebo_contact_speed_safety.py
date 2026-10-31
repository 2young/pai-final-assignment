"""Explicit fast-contact safety checks; moves/deletes box and pauses world.

Requires an updated running maze. Restart the maze after this test.
"""
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def service(name,kind,request):
    r=subprocess.run(['gz','service','-s',f'/world/maze/{name}','--reqtype',kind,
                      '--reptype','gz.msgs.Boolean','--timeout','2000','--req',request],
                     capture_output=True,text=True,timeout=5,check=True)
    assert 'data: true' in r.stdout,r.stdout+r.stderr


def place(name,x,y,z):
    service('set_pose','gz.msgs.Pose',
            f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}')


def main():
    rclpy.init();node=Node('fast_contact_safety_check');commands=[]
    sub=node.create_subscription(Twist,'/model/vehicle_blue/cmd_vel',
        lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)),100)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    process=None;paused=False
    try:
        for action in ('move','pause','interrupt','delete'):
            place('vehicle_blue',2.25,4,.325);place('red_box',5,4,.4)
            spin(.4);commands.clear()
            process=subprocess.Popen(['python3','/workspace/app/robot_control.py',
                                      '--goal-mode','contact','赤いオブジェクトまで行ってください'],
                                     stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            deadline=time.monotonic()+15
            while not any(v>=.35 for _,v,_ in commands) and time.monotonic()<deadline:
                if process.poll() is not None:raise RuntimeError(process.communicate()[0])
                spin(.03)
            assert any(v>=.35 for _,v,_ in commands),'Fast approach not reached'
            event=time.monotonic()
            if action=='move':
                place('red_box',5,5,.4);expected='object moved during navigation'
            elif action=='pause':
                service('control','gz.msgs.WorldControl','pause: true');paused=True
                expected='contact object pose missing or stale'
            elif action=='interrupt':
                process.send_signal(signal.SIGINT);expected='Interrupted'
            else:
                service('remove','gz.msgs.Entity','name: "red_box", type: MODEL')
                expected='contact object pose missing or stale'
            deadline=time.monotonic()+6
            while process.poll() is None and time.monotonic()<deadline:spin(.03)
            output=process.communicate(timeout=2)[0];spin(.1)
            if paused:
                service('control','gz.msgs.WorldControl','pause: false');paused=False
            print(output,flush=True)
            assert process.returncode in (1,130) and expected in output
            assert 'CONTACT_REACHED' not in output
            assert commands and commands[-1][1:]==(0.,0.)
            zeros=[t for t,v,w in commands if t>=event and (v,w)==(0.,0.)]
            assert zeros and min(zeros)-event<.8,zeros
            print(f'PASS: {action}, peak_command={max(v for _,v,_ in commands):.3f}m/s, '
                  f'first_zero_delay={min(zeros)-event:.3f}s, final_twist=(0.0,0.0)',flush=True)
            spin(.5)
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=5)
        if paused:service('control','gz.msgs.WorldControl','pause: false')
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
