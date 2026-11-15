#!/usr/bin/env bash
# Measures how much work the simulation is for this PC (no window).
#   bash scripts/measure_load.sh "label for this run"
# Prints: simulation speed, processor use of Gazebo, graphics card use and memory.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR"
source /opt/ros/jazzy/setup.bash
pkill -f "[h]elm.py"; pkill -f "[g]z sim"; pkill -f "[p]arameter_bridge"; sleep 2
BEFORE=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits)
(bash scripts/sim.sh headless > /tmp/wolf_sim.log 2>&1 &)
sleep 25
SUBS=$(gz model --list 2>/dev/null | grep -c "sub[0-9]")
RTF=$(timeout 5 gz topic -e -n 1 -t /stats | awk '/real_time_factor/ {printf "%.2f", $2}')
# average of 10 samples, one per second
GPU=0; MEM=0; CPU=0
for i in $(seq 10); do
  read -r u m <<< "$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits | tr -d ',')"
  c=$(top -b -n 1 | awk '/gz sim|ruby/ {s += $9} END {printf "%.0f", s}')
  GPU=$((GPU + u)); MEM=$((MEM + m)); CPU=$((CPU + c)); sleep 1
done
echo "$(date +%F) | ${1:-run} | subs=$SUBS | speed=$RTF | gazebo_cpu=$((CPU / 10))% of one core | gpu_use=$((GPU / 10))% | gpu_mem=$((MEM / 10)) MiB (was $BEFORE MiB before start)"
echo "renderer: $(grep -m1 -iE "GL_RENDERER|Selected|D3D12|llvmpipe|NVIDIA" ~/.gz/rendering/ogre2.log 2>/dev/null | cut -c1-120)"
echo "errors in Gazebo log: $(grep -ciE '\[Err' /tmp/wolf_sim.log)"
pkill -f "[g]z sim"
