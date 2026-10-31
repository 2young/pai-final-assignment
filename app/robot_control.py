"""ROS 2 node that translates Japanese text -> inferred intent -> Twist."""

import argparse
from pathlib import Path
import time
import joblib
from intent_model import MODEL_PATH, predict

# [forward speed m/s, angular speed rad/s, duration seconds]
MOTIONS = {
    "FORWARD": (1.2, 0.0, 2.8),
    "BACKWARD": (-0.7, 0.0, 2.0),
    "LEFT": (0.0, 0.8, 2.0),
    "RIGHT": (0.0, -0.8, 2.0),
    "STOP": (0.0, 0.0, 1.0),
}

DEMO = [
    "前へ移動してください", "右側に向きを変えて", "もう少し前に行って",
    "左に向いてください", "そのまま前進して", "ここで止めて",
]
TOPIC = "/model/vehicle_blue/cmd_vel"


def send_motion(node, pub, linear, angular, duration):
    from geometry_msgs.msg import Twist

    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        pub.publish(msg)
        time.sleep(0.1)
    # Explicit stop: never leave the robot moving between instructions.
    for _ in range(5):
        pub.publish(Twist())
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("commands", nargs="*", help="Japanese short commands")
    parser.add_argument("--dry-run", action="store_true", help="AI test without ROS/Gazebo")
    parser.add_argument("--demo", action="store_true", help="Use preselected instructions")
    args = parser.parse_args()
    if not Path(MODEL_PATH).is_file():
        raise SystemExit("Model not found. Run python3 app/train.py first.")
    clf = joblib.load(MODEL_PATH)
    commands = DEMO if args.demo else args.commands
    if not commands:
        raise SystemExit('Try: python3 app/robot_control.py --demo')
    results = [(text, predict(clf, text)) for text in commands]
    for text, intent in results:
        print(f"AI inference: {text!r} -> {intent}", flush=True)
    if args.dry_run:
        return

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node("japanese_instruction_robot")
    pub = node.create_publisher(Twist, TOPIC, 10)  # reliable QoS by default
    try:
        print("Waiting for ROS-Gazebo bridge subscription...", flush=True)
        for _ in range(100):
            if pub.get_subscription_count() > 0:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"No subscribers on {TOPIC}. Start Gazebo demo first.")
        for text, intent in results:
            linear, angular, duration = MOTIONS[intent]
            print(f"Executing {intent} (linear={linear}, angular={angular})", flush=True)
            send_motion(node, pub, linear, angular, duration)
    finally:
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
