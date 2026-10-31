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
  # ros2 / gz launch child processes; terminate their entire private groups.
  if [ -n "${sim_pid:-}" ]; then kill -INT -- "-$sim_pid" 2>/dev/null || true; fi
  if [ -n "${bridge_pid:-}" ]; then kill -TERM -- "-$bridge_pid" 2>/dev/null || true; fi
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# Set HEADLESS=1 for automated physics verification; GUI remains the default.
if [ "${HEADLESS:-0}" = 1 ]; then
  setsid gz sim -s -r /workspace/worlds/maze.sdf &
else
  setsid gz sim -r /workspace/worlds/maze.sdf &
fi
sim_pid=$!
# Use registry IDs for topics, matching the generated model names.
mapfile -t object_bridges < <(python3 - <<'PY'
import sys
sys.path.insert(0, '/workspace/app')
from grid_map import load_config
for obj in load_config()['objects']:
    print(f"/model/{obj['id']}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
    print(f"/model/{obj['id']}/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts")
PY
)
setsid ros2 run ros_gz_bridge parameter_bridge \
  '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/vehicle_blue/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  "${object_bridges[@]}" &
bridge_pid=$!
wait -n "$sim_pid" "$bridge_pid"
