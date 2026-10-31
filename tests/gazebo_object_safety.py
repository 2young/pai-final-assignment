"""Explicit integration check: requires a running maze; moves/deletes red_box.

Run after testing arrival, then restart the maze to restore the deleted box:
  docker compose exec sim bash -lc 'source /opt/ros/jazzy/setup.bash && python3 /workspace/tests/gazebo_object_safety.py'
Not collected by pytest. Uses the real CLI, physics, pose bridge and cmd_vel.
"""
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

COMMAND = ['python3', '/workspace/app/robot_control.py', '赤いオブジェクトまで行ってください']


def service(name, request_type, request):
    result = subprocess.run(['gz','service','-s',f'/world/maze/{name}',
                             '--reqtype',request_type,'--reptype','gz.msgs.Boolean',
                             '--timeout','2000','--req',request],
                            capture_output=True,text=True,timeout=5,check=True)
    if 'data: true' not in result.stdout:
        raise RuntimeError(result.stdout+result.stderr)


def place(x,y):
    service('set_pose','gz.msgs.Pose',
            f'name: "red_box", position: {{x: {x}, y: {y}, z: 0.4}}, orientation: {{w: 1}}')


def main():
    rclpy.init()
    node = Node('live_object_safety_check')
    messages = []
    subscription = node.create_subscription(
        Twist,'/model/vehicle_blue/cmd_vel',
        lambda m:messages.append((m.linear.x,m.angular.z)),100)
    process = None

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)

    def start():
        nonlocal process
        spin(.2)
        messages.clear()
        process = subprocess.Popen(COMMAND,stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT,text=True)

    def finished(expected, timeout=15, stationary=False):
        deadline = time.monotonic()+timeout
        while process.poll() is None and time.monotonic()<deadline:
            spin(.05)
        output = process.communicate(timeout=2)[0]
        spin(.2)
        print(output,flush=True)
        assert process.returncode == 1, process.returncode
        assert expected in output, output
        assert messages and messages[-1] == (0.,0.), messages[-5:]
        if stationary:
            assert all(m == (0.,0.) for m in messages), messages
        print(f'PASS: {expected}; final Twist={messages[-1]}',flush=True)

    def wait_for_motion():
        deadline = time.monotonic()+15
        while not any(m != (0.,0.) for m in messages) and time.monotonic()<deadline:
            if process.poll() is not None:
                raise RuntimeError(process.communicate()[0])
            spin(.05)
        assert any(m != (0.,0.) for m in messages), 'No motion command observed'

    try:
        for x,y,expected in [(0,-3,'overlaps'),(9,0,'outside field'),
                              (1,-3,'No reachable approach point')]:
            place(x,y)
            start()
            finished(expected,stationary=True)
        place(5,4)
        start()
        wait_for_motion()
        place(5,5)
        finished('object moved during navigation',timeout=6)
        place(5,4)
        start()
        wait_for_motion()
        service('remove','gz.msgs.Entity','name: "red_box", type: MODEL')
        finished('red_box: World pose missing or stale',timeout=6)
        start()
        finished('No fresh object world pose',timeout=14,stationary=True)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
