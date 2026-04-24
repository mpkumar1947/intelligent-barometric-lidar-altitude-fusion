#!/usr/bin/env bash
# End-to-end sanity check for the Gazebo -> ROS 2 ground-truth pipeline.
#
# Prerequisite: Gazebo is running (gz sim -v4 -r worlds/...) and the drone
# model is spawned.
#
# Tests, in order:
#   1. ros_gz_bridge is installed
#   2. Gazebo is publishing on the native topic
#   3. Start parameter_bridge (or skip if already running)
#   4. Confirm the bridged topic is visible from ROS 2
#   5. Start gz_pose_bridge.py
#   6. Confirm /gz/iris/pose is publishing
#
# Ctrl-C to abort.

set -u   # no set -e — we want to report failures, not exit on them
WORLD=${WORLD:-combined_drone_world}
MODEL=${MODEL:-iris_with_gimbal}

echo "=================================================="
echo "  gz ground-truth bridge smoke test"
echo "  world: $WORLD"
echo "  model: $MODEL"
echo "=================================================="

# 1 -----------------------------------------------------------------------
echo ""
echo "[1/6] ros_gz_bridge installed?"
if ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
    echo "      OK ($(ros2 pkg prefix ros_gz_bridge))"
else
    echo "      FAIL — install: sudo apt install ros-jazzy-ros-gz-bridge"
    exit 1
fi

# 2 -----------------------------------------------------------------------
echo ""
echo "[2/6] Gazebo publishing on /world/$WORLD/dynamic_pose/info?"
timeout 3 gz topic -l 2>/dev/null | grep -F "/world/$WORLD/dynamic_pose/info" >/dev/null
if [[ $? -eq 0 ]]; then
    echo "      OK"
else
    echo "      FAIL — check Gazebo is running and the world name is right."
    echo "      Try: gz topic -l | grep dynamic_pose"
    exit 1
fi

# 3 -----------------------------------------------------------------------
echo ""
echo "[3/6] starting ros_gz_bridge (background)..."
BRIDGE_TOPIC="/world/$WORLD/dynamic_pose/info"
BRIDGE_SPEC="${BRIDGE_TOPIC}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
if pgrep -f "parameter_bridge.*${BRIDGE_TOPIC}" >/dev/null; then
    echo "      SKIP (already running)"
    BRIDGE_PID=""
else
    ros2 run ros_gz_bridge parameter_bridge "$BRIDGE_SPEC" \
        > /tmp/gz_bridge.log 2>&1 &
    BRIDGE_PID=$!
    echo "      started PID $BRIDGE_PID (log: /tmp/gz_bridge.log)"
    sleep 2
fi

# 4 -----------------------------------------------------------------------
echo ""
echo "[4/6] bridged topic visible from ROS 2?"
if ros2 topic list 2>/dev/null | grep -Fq "$BRIDGE_TOPIC"; then
    echo "      OK"
else
    echo "      FAIL — check /tmp/gz_bridge.log"
    [[ -n "$BRIDGE_PID" ]] && kill "$BRIDGE_PID" 2>/dev/null
    exit 1
fi

# 5 -----------------------------------------------------------------------
echo ""
echo "[5/6] starting gz_pose_bridge.py (background)..."
if [[ ! -f gz_pose_bridge.py ]]; then
    echo "      FAIL — gz_pose_bridge.py not in CWD."
    echo "      cd into your src/ folder before running this."
    [[ -n "$BRIDGE_PID" ]] && kill "$BRIDGE_PID" 2>/dev/null
    exit 1
fi
python3 gz_pose_bridge.py \
    --ros-args \
    -p world_name:="$WORLD" \
    > /tmp/gz_pose_bridge.log 2>&1 &
POSE_PID=$!
echo "      started PID $POSE_PID (log: /tmp/gz_pose_bridge.log)"
sleep 3

# 6 -----------------------------------------------------------------------
echo ""
echo "[6/6] /gz/iris/pose publishing?"
RATE=$(timeout 4 ros2 topic hz /gz/iris/pose 2>&1 | grep "average rate" | head -1)
if [[ -n "$RATE" ]]; then
    echo "      OK  $RATE"
    echo ""
    echo "      sample message:"
    timeout 2 ros2 topic echo /gz/iris/pose --once 2>/dev/null | head -15
    echo ""
    echo "=================================================="
    echo "  OK — bridge works end-to-end."
    echo "  kill background procs when done:"
    echo "    kill $POSE_PID ${BRIDGE_PID:-"(no bridge pid)"}"
    echo "=================================================="
else
    echo "      FAIL — /gz/iris/pose not publishing."
    echo "      Check /tmp/gz_pose_bridge.log. Likely causes:"
    echo "        - drone not yet airborne (motion-detection needs movement)"
    echo "        - run inspect_gz_transforms.py and set manual_index"
    kill "$POSE_PID" 2>/dev/null
    [[ -n "$BRIDGE_PID" ]] && kill "$BRIDGE_PID" 2>/dev/null
    exit 1
fi
