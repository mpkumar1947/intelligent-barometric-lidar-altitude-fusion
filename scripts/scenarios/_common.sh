#!/usr/bin/env bash
# Shared helpers for all scenario launchers. Source this from run_*.sh.

# Wait until /mission/leg_info reports a leg_idx >= N. Channel [0] is leg_idx.
# Usage: wait_for_leg <min_idx> [max_wait_s]
wait_for_leg () {
    local target_idx=$1
    local max_wait=${2:-120}
    local t0=$(date +%s)

    echo "[scenario] waiting for leg >= $target_idx (max ${max_wait}s)"
    while : ; do
        local line
        line=$(timeout 2 ros2 topic echo /mission/leg_info std_msgs/msg/Float64MultiArray --once 2>/dev/null | grep -A1 'data:' | tail -1 || true)
        local idx
        idx=$(echo "$line" | grep -oE '[0-9]+\.[0-9]+' | head -1)
        if [[ -n "$idx" ]]; then
            local iint=${idx%.*}
            if (( iint >= target_idx )); then
                echo "[scenario] leg $iint reached"
                return 0
            fi
        fi
        local now=$(date +%s)
        if (( now - t0 > max_wait )); then
            echo "[scenario] WARN: timed out waiting for leg $target_idx"
            return 1
        fi
        sleep 1
    done
}

# Wait for mission completion by watching for mission_fusion.png, which
# mission_node writes in _generate_plots at DONE. More reliable than
# polling /mission/leg_info for silence — leg_info stops at LAND_PHASE,
# which can be 30-40s before actual completion due to the LAND descent.
wait_for_mission_done () {
    local max_wait=${1:-900}
    local t0=$(date +%s)

    # Delete any stale plot — otherwise we'd get an instant false positive.
    rm -f mission_fusion.png mission_3d.png

    echo "[scenario] waiting for mission plot (max ${max_wait}s)"
    while [[ ! -f mission_fusion.png ]]; do
        local now=$(date +%s)
        if (( now - t0 > max_wait )); then
            echo "[scenario] WARN: timeout, no plot produced"
            return 1
        fi
        sleep 2
    done

    # Give mission_node a beat to finish writing mission_3d.png.
    sleep 3
    echo "[scenario] mission complete"
    return 0
}

set_param () {
    local node=$1 key=$2 val=$3
    echo "[scenario] ros2 param set $node $key $val"
    ros2 param set "$node" "$key" "$val"
}

wait_for_file () {
    local path=$1
    local max_wait=${2:-30}
    local t0=$(date +%s)
    while [[ ! -f "$path" ]]; do
        local now=$(date +%s)
        if (( now - t0 > max_wait )); then
            echo "[scenario] WARN: $path not produced in ${max_wait}s"
            return 1
        fi
        sleep 1
    done
    return 0
}

# Graceful shutdown: SIGINT the process group (not just the PID — mission_node
# runs behind a `| tee` pipe, and killing just the Python PID leaves tee
# holding stdout). Must never fail under set -e.
stop_mission () {
    local pid=$1
    local max_wait=${2:-15}
    set +e
    if ! kill -0 "$pid" 2>/dev/null; then
        set -e
        return 0
    fi
    echo "[scenario] SIGINT -> $pid"
    kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    local t0 now
    t0=$(date +%s)
    while kill -0 "$pid" 2>/dev/null; do
        now=$(date +%s)
        if (( now - t0 > max_wait )); then
            echo "[scenario] WARN: didn't exit in ${max_wait}s — SIGKILL"
            kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
            break
        fi
        sleep 1
    done
    wait "$pid" 2>/dev/null
    set -e
    return 0
}
