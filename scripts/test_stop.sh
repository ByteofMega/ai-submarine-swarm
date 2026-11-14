#!/usr/bin/env bash
# Test: the stop command, resume, the "no position" safety rule, and the stop on shutdown.
# Starts everything itself (no window) and writes the helm log to logs/stop_test.log
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"
source /opt/ros/jazzy/setup.bash
mkdir -p logs
LOG=logs/stop_test.log

stop_all() { pkill -f "[g]z sim"; pkill -f "[p]arameter_bridge"; pkill -f "[h]elm.py"; }
depth() { timeout 5 ros2 topic echo /sub1/odom --once --field pose.pose.position.z | head -1 | awk '{printf "%.2f", -$1}'; }
# How fast the front vertical propeller spins, read straight from Gazebo (0 = off)
prop() { timeout 5 gz topic -e -n 1 -t /model/sub1/joint/vert_front_joint/ang_vel | awk '/data/ {printf "%.1f", $2} END {if (NR==0) printf "0.0"}'; }
say() { echo; echo "### $1"; }
pause_sim() { gz service -s /world/lrauv_demo/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 2000 --req "pause: $1" > /dev/null; }

stop_all; sleep 1
(bash scripts/sim.sh headless > /tmp/wolf_sim.log 2>&1 &)
sleep 12
(bash scripts/bridge.sh > /tmp/wolf_bridge.log 2>&1 &)
sleep 4
(python3 -u scripts/helm.py --ros-args -r __ns:=/sub1 > "$LOG" 2>&1 &)
sleep 6

say "1. Dive to 6 m and hold"
ros2 topic pub --once /sub1/helm/target_depth std_msgs/msg/Float64 "{data: 6.0}" > /dev/null
sleep 25
echo "depth $(depth) m, propeller speed $(prop)"

say "2. STOP command"
ros2 topic pub --once /sub1/helm/stop std_msgs/msg/Bool "{data: true}" > /dev/null
sleep 3
D1=$(depth); echo "3 s after stop:  depth $D1 m, propeller speed $(prop)"
echo "helm state: $(timeout 5 ros2 topic echo /sub1/helm/state --once --field data | head -1)"
sleep 10
D2=$(depth); echo "13 s after stop: depth $D2 m, propeller speed $(prop)  (floating up slowly is expected)"

say "3. A new target while stopped must be ignored"
ros2 topic pub --once /sub1/helm/target_depth std_msgs/msg/Float64 "{data: 15.0}" > /dev/null
sleep 5
echo "depth $(depth) m, propeller speed $(prop)"

say "4. RESUME: hold the depth it is at now"
ros2 topic pub --once /sub1/helm/stop std_msgs/msg/Bool "{data: false}" > /dev/null
sleep 10
echo "depth $(depth) m, helm state: $(timeout 5 ros2 topic echo /sub1/helm/state --once --field data | head -1)"

say "5. Position data stops (simulation paused for 4 s): helm must switch thrusters off"
pause_sim true; sleep 4
echo "helm state while paused: $(timeout 5 ros2 topic echo /sub1/helm/state --once --field data | head -1)"
pause_sim false; sleep 6
echo "after un-pausing: depth $(depth) m, helm state: $(timeout 5 ros2 topic echo /sub1/helm/state --once --field data | head -1)"

say "6. Dive to 8 m, then end the helm program (like Ctrl+C) while it is pushing"
ros2 topic pub --once /sub1/helm/target_depth std_msgs/msg/Float64 "{data: 8.0}" > /dev/null
sleep 2
echo "before: propeller speed $(prop)"
pkill -INT -f "[h]elm.py"; sleep 3
echo "after:  propeller speed $(prop), helm still running: $(pgrep -f '[h]elm.py' > /dev/null && echo yes || echo no)"

stop_all
say "Helm log lines about stop, resume, safety and shutdown ($LOG)"
grep -E "STOP received|RESUME|ignored|no position|shutdown|confirmed|ERROR|Traceback" "$LOG" | cut -c 44-
