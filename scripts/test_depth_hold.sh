#!/usr/bin/env bash
# Test: sub1 must reach and hold 5 m, then 12 m, then 3 m, then stop above the seabed.
# Starts everything itself (no window) and writes the helm log to logs/depth_hold_test.log
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"
source /opt/ros/jazzy/setup.bash
mkdir -p logs
LOG=logs/depth_hold_test.log

stop_all() { pkill -f "[g]z sim"; pkill -f "[p]arameter_bridge"; pkill -f "[h]elm.py"; }
stop_all; sleep 1
(bash scripts/sim.sh headless > /tmp/wolf_sim.log 2>&1 &)
sleep 12
(bash scripts/bridge.sh > /tmp/wolf_bridge.log 2>&1 &)
sleep 4
(python3 -u scripts/helm.py --ros-args -r __ns:=/sub1 > "$LOG" 2>&1 &)
sleep 6

go() {   # go <depth> <seconds to wait>
  echo "target $1 m, waiting $2 s ..."
  ros2 topic pub --once /sub1/helm/target_depth std_msgs/msg/Float64 "{data: $1}" > /dev/null
  sleep "$2"
}
go 5.0 40
go 12.0 45
go 3.0 45
go 40.0 75      # deeper than the sea: the altimeter must stop it above the seabed
stop_all

echo
echo "=== Result: last 10 seconds before each new target ==="
python3 - "$LOG" <<'EOF'
import re, sys
rows, blocks = [], []
for line in open(sys.argv[1]):
    if 'new target depth' in line:
        blocks.append(rows); rows = []
    m = re.search(r'depth=\s*([\d.-]+) target=\s*([\d.-]+) error=([+\d.-]+).*alt=(\S+)', line)
    if m:
        rows.append((float(m[1]), float(m[2]), float(m[3]), m[4]))
blocks.append(rows)
for rows in blocks[1:]:
    last = rows[-10:]
    worst = max(abs(r[2]) for r in last)
    print('target %6.2f m: depth %.2f .. %.2f m, worst error %.2f m, altitude %s -> %s' % (
        last[-1][1], min(r[0] for r in last), max(r[0] for r in last), worst, last[-1][3],
        'OK' if worst < 0.10 else 'NOT OK'))
EOF
echo "Full log: $LOG"
