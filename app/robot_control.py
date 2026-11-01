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
    parser.add_argument('--controller',choices=['waypoint','pure_pursuit'],help='Override path follower')
    parser.add_argument('--planner',choices=['auto','circle','hybrid'],help='Override path planner')
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
    from grid_map import load_config
    from target_resolver import resolve_target
    from path_planner import route_score
    from adaptive_planner import adaptive_plan
    from boundary_entry import plan_boundary_entry,execute_boundary_entry
    from contact_goal import footprint_free
    from pose_provider import angle_difference
    from ros_feedback import spin_some
    config = load_config()
    if args.planner:config['planner_mode']=args.planner
    if args.controller:
        config.setdefault('path_following',{})['controller']=args.controller
    from follower_config import FollowerSettings
    follower_settings=FollowerSettings.from_config(config)
    if follower_settings.controller=='waypoint' and config.get('planner_mode','circle')!='circle':
        if args.planner is not None:
            raise ValueError('Waypoint controller requires --planner circle; pose routes require pure_pursuit')
        config['planner_mode']='circle'
    from pose_planner import PoseSettings
    PoseSettings.from_config(config)
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
            navigation_guard=(lambda:tracker.assert_unchanged(max_age=follower_settings.pose_timeout)
                              if follower_settings.controller=='pure_pursuit' else tracker.assert_unchanged())
            navigator = Navigator(node, pub, None, config=config)
            if goal_mode == 'contact' and targets:
                from motion_config import ContactSettings
                ContactSettings.from_config(config)  # Validate before any motion.
                from contact_goal import ContactMonitor, touch, can_resume_touch
                from contact_speed import FEEDBACK_WALL_TIMEOUT
                contact_guard=lambda:tracker.assert_unchanged(max_age=FEEDBACK_WALL_TIMEOUT)
                contacts = {o['id']: ContactMonitor(node,o['id']) for o in targets.values()}
                for monitor in contacts.values():
                    monitor.require_sensor()

            def prepare_navigation(touch_target=None):
                live = tracker.snapshot()
                navigator.planning_pose = navigator.current()
                start = navigator.planning_pose[:2]
                tracker.assert_unchanged()
                if not footprint_free(live,navigator.planning_pose,touch_target):
                    raise ValueError('Robot footprint violates current obstacle / operating area clearance; reposition robot or obstacle')
                return live, start

            def validate_plan_start(start):
                # Planning can take longer than a pose's freshness window.
                # Refresh observations without resetting the target baseline.
                requested=time.monotonic()
                providers=[navigator.provider,*tracker.providers.values()]
                while not all(p.received>=requested for p in providers):
                    if time.monotonic()-requested>3:
                        raise RuntimeError('No fresh pose after planning')
                    spin_some(node,timeout_sec=.05)
                tracker.assert_unchanged()
                current=navigator.provider.current()
                if (math.dist(current[:2],start)>.05 or
                        abs(angle_difference(current[2],navigator.planning_pose[2]))>math.radians(3)):
                    raise RuntimeError('Robot moved while planning; issue command again')

        if args.goal is not None:
            live, start = prepare_navigation()
            yaw=navigator.planning_pose[2]
            entry=(plan_boundary_entry(live,navigator.planning_pose)
                   if live.get('planner_mode','circle')=='circle' else None)
            route_start=entry['goal'] if entry else start
            navigator.grid,route=adaptive_plan(live,route_start,goal=tuple(args.goal),start_yaw=yaw)
            score=route_score(route,yaw)
            length = score[1]
            print(f"A*: start={start}, goal={args.goal}, length={length:.2f}m, "
                  f"planned_turns={score[0]}, waypoints={route}", flush=True)
            validate_plan_start(start)
            if entry:
                execute_boundary_entry(navigator,live,entry,lambda:tracker.assert_unchanged(max_age=.3))
            navigator.follow(route, guard=navigation_guard)
        for text, intent in results:
            if intent == "GO_TO_OBJECT":
                live, start = prepare_navigation(targets[text]['id'] if goal_mode=='contact' else None)
                yaw=navigator.planning_pose[2]
                target = next(o for o in live['objects'] if o['id'] == targets[text]['id'])
                if goal_mode == 'contact' and contacts[target['id']].subscription.get_publisher_count() == 0:
                    raise RuntimeError('No ROS contact bridge; restart maze')
                if goal_mode == 'contact' and can_resume_touch(live,navigator.provider.current(),target):
                    touch(navigator,live,target,contacts[target['id']],contact_guard)
                    continue
                entry=(plan_boundary_entry(live,navigator.planning_pose)
                       if live.get('planner_mode','circle')=='circle' else None)
                route_start=entry['goal'] if entry else start
                navigator.grid,route=adaptive_plan(live,route_start,target=target,start_yaw=yaw,
                                                   contact=goal_mode=='contact')
                final_yaw=(math.atan2(target['center'][1]-route[-1][1],
                                     target['center'][0]-route[-1][0]) if goal_mode=='contact' else None)
                score=route_score(route,yaw,final_yaw)
                length = score[1]
                print(f"A*: start={start}, goal={route[-1]}, target={targets[text]['id']}, "
                      f"length={length:.2f}m, planned_turns={score[0]}, "
                      f"planned_yaw={math.degrees(score[2]):.1f}deg, waypoints={route}", flush=True)
                validate_plan_start(start)
                if entry:
                    execute_boundary_entry(navigator,live,entry,contact_guard if goal_mode=='contact'
                                           else lambda:tracker.assert_unchanged(max_age=.3))
                phases=[navigator.follow(route, guard=navigation_guard)]
                if goal_mode == 'contact':
                    phases.append(touch(navigator,live,target,contacts[target['id']],contact_guard))
                print(f"NAVIGATION_SUMMARY: planned_turns={score[0]}, "
                      f"rotation_episodes={sum(p['rotation_episodes'] for p in phases)}, "
                      f"total_yaw={sum(p['total_yaw_degrees'] for p in phases):.1f}deg",flush=True)
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
