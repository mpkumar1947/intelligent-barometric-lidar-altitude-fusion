#!/usr/bin/env bash
# Slow systematic baro drift + 50s-period oscillation.
# The paper's headline stress test: drift is a bias, not a variance, so
# adaptive-R can't detect it. Stage-1 diverges; Stage-2A/2B lean on LiDAR.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

SCENARIO=drift
TS=$(date +%Y%m%d_%H%M%S)
mkdir -p logs outputs

echo "[scenario] === $SCENARIO @ $TS ==="

set_param /sensor_rate_manager disturbance_mode           BARO_DRIFT
set_param /sensor_rate_manager disturbance_active         False
set_param /sensor_rate_manager baro_drift_rate_pa_per_s   5.0
set_param /sensor_rate_manager baro_drift_oscillation_hz  0.02
set_param /sensor_rate_manager baro_drift_osc_amp_pa      40.0

export SCENARIO TS
python3 mission_node.py 2>&1 | tee "logs/${SCENARIO}_${TS}.log" &
MPID=$!

wait_for_leg 2 120 || true
echo "[scenario] arming BARO_DRIFT"
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
