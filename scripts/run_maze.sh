#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
source_sdf=$(find /usr/share /opt/ros/jazzy/opt -path '*/worlds/diff_drive.sdf' -print -quit)
if [ -z "$source_sdf" ]; then
  echo 'Gazebo diff_drive.sdf not found' >&2
  exit 1
fi
python3 /workspace/app/generate_world.py --source "$source_sdf"
cleanup() {
  kill "${sim_pid:-}" "${bridge_pid:-}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# Set HEADLESS=1 for automated physics verification; GUI remains the default.
if [ "${HEADLESS:-0}" = 1 ]; then
  gz sim -s -r /workspace/worlds/maze.sdf &
else
  gz sim -r /workspace/worlds/maze.sdf &
fi
sim_pid=$!
ros2 run ros_gz_bridge parameter_bridge \
  '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/vehicle_blue/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' &
bridge_pid=$!
wait -n "$sim_pid" "$bridge_pid"
