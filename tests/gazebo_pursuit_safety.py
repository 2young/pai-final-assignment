"""High-speed path tracking stop checks; restart maze after deleting a wall."""
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_contact_speed_safety import service,place


def main():
    rclpy.init();node=Node('pursuit_safety_check');commands=[]
    sub=node.create_subscription(Twist,'/model/vehicle_blue/cmd_vel',
        lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)),100)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    process=None;paused=False
    try:
        for action in ('move','pause','interrupt','delete'):
            place('vehicle_blue',-4.5,5,.325);place('red_box',5,5,.4)
            place('west_wall',-8,0,.5)
            spin(.4);commands.clear()
            process=subprocess.Popen(['python3','/workspace/app/robot_control.py',
                '赤いオブジェクトまで行ってください'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            deadline=time.monotonic()+20
            while not any(v>=.8 for _,v,_ in commands) and time.monotonic()<deadline:
                if process.poll() is not None:raise RuntimeError(process.communicate()[0])
                spin(.02)
            assert any(v>=.8 for _,v,_ in commands),'Fast path speed not reached'
            event=time.monotonic()
            if action=='move':
                place('west_wall',-8,1,.5);expected='object moved during navigation'
            elif action=='pause':
                service('control','gz.msgs.WorldControl','pause: true');paused=True
                expected='pose missing or stale'
            elif action=='interrupt':
                process.send_signal(signal.SIGINT);expected='Interrupted'
            else:
                service('remove','gz.msgs.Entity','name: "west_wall", type: MODEL')
                expected='pose missing or stale'
            deadline=time.monotonic()+6
            while process.poll() is None and time.monotonic()<deadline:spin(.02)
            output=process.communicate(timeout=2)[0];spin(.2)
            if paused:service('control','gz.msgs.WorldControl','pause: false');paused=False
            print(output,flush=True)
            assert process.returncode in (1,130) and expected in output
            assert 'CONTACT_REACHED' not in output and commands[-1][1:]==(0.,0.)
            zeros=[t for t,v,w in commands if t>=event and (v,w)==(0.,0.)]
            assert zeros and min(zeros)-event<.8
            print(f'PASS: {action}, first_zero={min(zeros)-event:.3f}s, peak={max(v for _,v,_ in commands):.3f}m/s',flush=True)
            spin(.5)
    finally:
        if process is not None and process.poll() is None:process.terminate();process.wait(timeout=5)
        if paused:service('control','gz.msgs.WorldControl','pause: false')
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
