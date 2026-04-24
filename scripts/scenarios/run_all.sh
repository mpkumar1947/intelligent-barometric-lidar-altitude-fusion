#!/usr/bin/env bash
# Run all 5 scenarios back-to-back.
#
# Assumes Gazebo + ArduPilot SITL + MAVROS + the support nodes are already
# running. Only mission_node is launched per scenario. 20s pause between
# runs lets SITL / MAVROS settle for the next arm.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SCENARIOS=(
    run_nominal.sh
    run_wind.sh
    run_drift.sh
    run_dropout.sh
    run_combined.sh
)

for s in "${SCENARIOS[@]}"; do
    echo ""
    echo "#####################################"
    echo "#  $s"
    echo "#####################################"
    bash "$SCRIPT_DIR/$s" || echo "[run_all] $s FAILED — continuing"
    echo "[run_all] pause 20s..."
    sleep 20
done

echo ""
echo "[run_all] all scenarios attempted. Outputs in ./outputs/, logs in ./logs/"
