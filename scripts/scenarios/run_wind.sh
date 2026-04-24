#!/usr/bin/env bash
# Baro noise scenario (random walk + Gaussian).
# Armed once the drone is past takeoff (leg idx >= 2).
# Expectation: Stage-1 degrades visibly; Stage-2A/2B ride out via LiDAR.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

SCENARIO=wind
TS=$(date +%Y%m%d_%H%M%S)
mkdir -p logs outputs

echo "[scenario] === $SCENARIO @ $TS ==="

# Configure mode before launching. disturbance_active is flipped post-takeoff.
set_param /sensor_rate_manager disturbance_mode         WIND
set_param /sensor_rate_manager disturbance_active       False
set_param /sensor_rate_manager wind_baro_rw_sigma_pa    8.0
set_param /sensor_rate_manager wind_baro_gauss_sigma_pa 15.0
set_param /sensor_rate_manager wind_baro_rw_decay       0.995

export SCENARIO TS
python3 mission_node.py 2>&1 | tee "logs/${SCENARIO}_${TS}.log" &
MPID=$!

wait_for_leg 2 120 || true
echo "[scenario] arming WIND"
set_param /sensor_rate_manager disturbance_active True

wait_for_mission_done 900 || true
stop_mission "$MPID" 15
wait_for_file mission_fusion.png 30 || true

# Clean state for the next run.
set_param /sensor_rate_manager disturbance_active False || true
set_param /sensor_rate_manager disturbance_mode   NONE  || true

ls -la mission_fusion.png mission_3d.png 2>/dev/null || echo "  (no plots in CWD)"
[[ -f mission_fusion.png ]] && mv mission_fusion.png "outputs/mission_fusion_${SCENARIO}_${TS}.png"
[[ -f mission_3d.png     ]] && mv mission_3d.png     "outputs/mission_3d_${SCENARIO}_${TS}.png"

echo "[scenario] $SCENARIO done."
