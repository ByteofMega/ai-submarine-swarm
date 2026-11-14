#!/usr/bin/env bash
# Test: sub1 and sub2 hold different depths at the same time, and stopping one
# does not affect the other. Logs go to logs/two_subs_sub1.log and logs/two_subs_sub2.log
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"
source /opt/ros/jazzy/setup.bash
mkdir -p logs

stop_all() { pkill -INT -f "[h]elm.py"; sleep 1; pkill -f "[g]z sim"; pkill -f "[p]arameter_bridge"; }
depth() { timeout 5 ros2 topic echo /$1/odom --once --field pose.pose.position.z | head -1 | awk '{printf "%.2f", -$1}'; }
state() { timeout 5 ros2 topic echo /$1/helm/state --once --field data | head -1; }
say() { echo; echo "### $1"; }

pkill -f "[h]elm.py"; pkill -f "[g]z sim"; pkill -f "[p]arameter_bridge"; sleep 1
(bash scripts/sim.sh headless > /tmp/wolf_sim.log 2>&1 &)
sleep 14
(bash scripts/bridge.sh sub1 sub2 > /tmp/wolf_bridge.log 2>&1 &)
sleep 4
for sub in sub1 sub2; do
  (python3 -u scripts/helm.py --ros-args -r __ns:=/$sub > logs/two_subs_$sub.log 2>&1 &)
done
sleep 6

say "1. sub1 to 4 m, sub2 to 9 m"
ros2 topic pub --once /sub1/helm/target_depth std_msgs/msg/Float64 "{data: 4.0}" > /dev/null
ros2 topic pub --once /sub2/helm/target_depth std_msgs/msg/Float64 "{data: 9.0}" > /dev/null
sleep 35
echo "sub1: depth $(depth sub1) m, state $(state sub1)"
echo "sub2: depth $(depth sub2) m, state $(state sub2)"

say "2. Stop sub1 only, wait 10 s"
ros2 topic pub --once /sub1/helm/stop std_msgs/msg/Bool "{data: true}" > /dev/null
sleep 10
echo "sub1: depth $(depth sub1) m, state $(state sub1)   (floating up is expected)"
echo "sub2: depth $(depth sub2) m, state $(state sub2)   (must still be at 9 m)"

say "3. Simulation speed (1.00 = real time)"
timeout 5 gz topic -e -n 1 -t /stats | awk '/real_time_factor/ {printf "real time factor: %.2f\n", $2}'

stop_all
say "Last line of each helm log"
tail -n 1 logs/two_subs_sub1.log | cut -c 44-
tail -n 1 logs/two_subs_sub2.log | cut -c 44-
echo "warnings or errors in the logs: $(cat logs/two_subs_sub*.log | grep -cE '\[WARN\]|\[ERROR\]|Traceback')"
