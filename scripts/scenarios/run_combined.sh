#!/usr/bin/env bash
# Worst-case: wind + dropout + drift simultaneously.
# Parameters are gentler than the single-fault scripts — the compound
# effect otherwise triggers repeated failsafes.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

SCENARIO=combined
TS=$(date +%Y%m%d_%H%M%S)
mkdir -p logs outputs

echo "[scenario] === $SCENARIO @ $TS ==="

set_param /sensor_rate_manager disturbance_mode   COMBINED
set_param /sensor_rate_manager disturbance_active False

set_param /sensor_rate_manager wind_baro_rw_sigma_pa     5.0
set_param /sensor_rate_manager wind_baro_gauss_sigma_pa  10.0
set_param /sensor_rate_manager lidar_dropout_burst_prob  0.01
set_param /sensor_rate_manager lidar_dropout_burst_len   15
set_param /sensor_rate_manager lidar_spike_prob          0.003
set_param /sensor_rate_manager baro_drift_rate_pa_per_s  3.0
set_param /sensor_rate_manager baro_drift_osc_amp_pa     25.0

export SCENARIO TS
python3 mission_node.py 2>&1 | tee "logs/${SCENARIO}_${TS}.log" &
MPID=$!

wait_for_leg 2 120 || true
echo "[scenario] arming COMBINED"
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
