#!/usr/bin/env bash
# LiDAR dropout bursts + spurious spikes.
# Default burst length (25 samples ≈ 0.25s) stays below the 3s
# LIDAR_FAILED threshold, so we see LIDAR_DEGRADED but not full failsafe.
# For an actual failsafe trip, crank burst length to ~400 samples.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

SCENARIO=dropout
TS=$(date +%Y%m%d_%H%M%S)
mkdir -p logs outputs

echo "[scenario] === $SCENARIO @ $TS ==="

set_param /sensor_rate_manager disturbance_mode          LIDAR_DROPOUT
set_param /sensor_rate_manager disturbance_active        False
set_param /sensor_rate_manager lidar_dropout_burst_prob  0.02
set_param /sensor_rate_manager lidar_dropout_burst_len   25
set_param /sensor_rate_manager lidar_spike_prob          0.005
set_param /sensor_rate_manager lidar_spike_magnitude_m   30.0

export SCENARIO TS
python3 mission_node.py 2>&1 | tee "logs/${SCENARIO}_${TS}.log" &
MPID=$!

wait_for_leg 2 120 || true
echo "[scenario] arming LIDAR_DROPOUT"
set_param /sensor_rate_manager disturbance_active True

wait_for_mission_done 900 || true
stop_mission "$MPID" 15
wait_for_file mission_fusion.png 30 || true

set_param /sensor_rate_manager disturbance_active False || true
set_param /sensor_rate_manager disturbance_mode   NONE  || true

ls -la mission_fusion.png mission_3d.png 2>/dev/null || echo "  (no plots in CWD)"
[[ -f mission_fusion.png ]] && mv mission_fusion.png "outputs/mission_fusion_${SCENARIO}_${TS}.png"
[[ -f mission_3d.png     ]] && mv mission_3d.png     "outputs/mission_3d_${SCENARIO}_${TS}.png"

echo "[scenario] $SCENARIO done."

# To force LIDAR_FAILED trip: raise burst length to ~400 samples (4s at 100Hz):
#   set_param /sensor_rate_manager lidar_dropout_burst_len 400
