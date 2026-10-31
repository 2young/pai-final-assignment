"""ROS 2 node that translates Japanese text -> inferred intent -> Twist."""

import argparse
import math
import sys
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
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--goal-mode", choices=['contact','stand_off'],
                        help="Object goal: sensor-confirmed touch or stand-off")
    args = parser.parse_args()
    if not Path(MODEL_PATH).is_file():
        raise SystemExit("Model not found. Run python3 app/train.py first.")
    clf = joblib.load(MODEL_PATH)
    if "GO_TO_OBJECT" not in clf.classes_:
        raise SystemExit("Old model: run python3 app/train.py to retrain for navigation.")
    commands = DEMO if args.demo else args.commands
    if not commands and args.goal is None:
        raise SystemExit('Try: python3 app/robot_control.py --demo')
    results = [(text, predict(clf, text)) for text in commands]
    for text, intent in results:
        print(f"AI inference: {text!r} -> {intent}", flush=True)
    from grid_map import GridMap, load_config
    from target_resolver import resolve_target, plan_to_object
    from path_planner import plan
    config = load_config()
    goal_mode = args.goal_mode or config.get('goal_mode','stand_off')
    if goal_mode not in ('contact','stand_off'):
        raise ValueError('goal_mode must be contact or stand_off')
    # Resolve before starting any motion; a failed command cancels the batch.
    try:
        targets = {text: resolve_target(text, config) for text, intent in results
                   if intent == "GO_TO_OBJECT"}
    except ValueError as exc:
        if args.dry_run:
            raise SystemExit(str(exc))
        targets = None  # Enter ROS finally block to publish STOP before reporting.
    if args.dry_run:
        for text, target in targets.items():
            print(f"Registered target: {target['id']} (initial position={target['center']}; "
                  "live position is obtained only during ROS execution)")
        return

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    # Keep the ROS context alive while finally publishes STOP on Ctrl+C.
    from rclpy.signals import SignalHandlerOptions
    import signal
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
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
        if targets is None:
            raise ValueError("Target unknown or ambiguous; stopped")
        if args.goal is not None or targets:
            from navigator import Navigator
            from object_tracker import ObjectTracker
            tracker = ObjectTracker(node, config)
            navigator = Navigator(node, pub, None, config=config)
            if goal_mode == 'contact' and targets:
                from contact_goal import ContactMonitor, plan_contact, touch, can_resume_touch
                contacts = {o['id']: ContactMonitor(node,o['id']) for o in targets.values()}
                for monitor in contacts.values():
                    monitor.require_sensor()

            def prepare_navigation():
                live = tracker.snapshot()
                navigator.grid = GridMap(live)
                start = navigator.current()[:2]
                tracker.assert_unchanged()
                return live, start

        if args.goal is not None:
            _, start = prepare_navigation()
            route = plan(navigator.grid, start, tuple(args.goal))
            length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
            print(f"A*: start={start}, goal={args.goal}, length={length:.2f}m, waypoints={route}", flush=True)
            navigator.follow(route, guard=tracker.assert_unchanged)
        for text, intent in results:
            if intent == "GO_TO_OBJECT":
                live, start = prepare_navigation()
                target = next(o for o in live['objects'] if o['id'] == targets[text]['id'])
                if goal_mode == 'contact' and contacts[target['id']].subscription.get_publisher_count() == 0:
                    raise RuntimeError('No ROS contact bridge; restart maze')
                if (goal_mode == 'contact' and not navigator.grid.free(navigator.grid.cell(start))
                        and can_resume_touch(live,navigator.provider.current(),target)):
                    touch(navigator,live,target,contacts[target['id']],tracker.assert_unchanged)
                    continue
                route = (plan_contact(navigator.grid,start,target) if goal_mode == 'contact'
                         else plan_to_object(navigator.grid,start,target))
                length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
                print(f"A*: start={start}, goal={route[-1]}, target={targets[text]['id']}, "
                      f"length={length:.2f}m, waypoints={route}", flush=True)
                navigator.follow(route, guard=tracker.assert_unchanged)
                if goal_mode == 'contact':
                    touch(navigator,live,target,contacts[target['id']],tracker.assert_unchanged)
                continue
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
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        raise SystemExit(130)
    except (ValueError, RuntimeError) as exc:
        print(f"Aborted: {exc}", file=sys.stderr)
        raise SystemExit(1)
