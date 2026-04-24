#!/usr/bin/env bash
# Baseline scenario. No disturbances.
#
# Assumes Gazebo + ArduPilot SITL + MAVROS are running, and the support
# nodes (sensor_rate_manager, altitude_fusion_node, failsafe_node,
# gz_pose_bridge) are already up. This script starts mission_node only.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

SCENARIO=nominal
TS=$(date +%Y%m%d_%H%M%S)
mkdir -p logs outputs

echo "[scenario] === $SCENARIO @ $TS ==="

# Explicitly disarm disturbances — previous runs may have left them on.
set_param /sensor_rate_manager disturbance_mode   NONE
set_param /sensor_rate_manager disturbance_active False

export SCENARIO TS   # mission_node reads these to tag its pickle
python3 mission_node.py 2>&1 | tee "logs/${SCENARIO}_${TS}.log" &
MPID=$!

wait_for_mission_done 900 || true
stop_mission "$MPID" 15
wait_for_file mission_fusion.png 30 || true

ls -la mission_fusion.png mission_3d.png 2>/dev/null || echo "  (no plots in CWD)"
[[ -f mission_fusion.png ]] && mv mission_fusion.png "outputs/mission_fusion_${SCENARIO}_${TS}.png"
[[ -f mission_3d.png     ]] && mv mission_3d.png     "outputs/mission_3d_${SCENARIO}_${TS}.png"

echo "[scenario] $SCENARIO done."
