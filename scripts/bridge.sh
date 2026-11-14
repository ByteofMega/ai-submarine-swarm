#!/usr/bin/env bash
# Starts the bridge that passes messages between Gazebo and ROS 2.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$DIR/config/bridge.yaml"
