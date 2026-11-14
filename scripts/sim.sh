#!/usr/bin/env bash
# Starts the wolf pack world in Gazebo.
#   bash scripts/sim.sh            with the 3D window
#   bash scripts/sim.sh headless   without a window (for tests)
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
# Tell Gazebo where the wolfsub model is
export GZ_SIM_RESOURCE_PATH="$DIR/models:${GZ_SIM_RESOURCE_PATH:-}"
if [ "$1" = "headless" ]; then
  exec gz sim -s -r --headless-rendering "$DIR/worlds/wolfpack.sdf"
else
  exec gz sim -r "$DIR/worlds/wolfpack.sdf"
fi
