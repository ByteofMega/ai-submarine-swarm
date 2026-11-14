#!/usr/bin/env bash
# Starts the bridge that passes messages between Gazebo and ROS 2.
#   bash scripts/bridge.sh              for sub1 and sub2
#   bash scripts/bridge.sh sub1         for sub1 only
#   bash scripts/bridge.sh sub1 sub2 sub3
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
SUBS="${*:-sub1 sub2}"

# Build one config file from the template: one copy of the topic list per sub
CONFIG=/tmp/wolfpack_bridge.yaml
: > "$CONFIG"
for sub in $SUBS; do
  grep -v "^#" "$DIR/config/bridge_template.yaml" | sed "s/SUBNAME/$sub/g" >> "$CONFIG"
done
echo "bridge for: $SUBS  (config: $CONFIG)"
exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$CONFIG"
