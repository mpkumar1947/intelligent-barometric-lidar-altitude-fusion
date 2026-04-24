#!/usr/bin/env python3
"""
Gazebo pose bridge.

Republishes the simulator's ground-truth pose for the drone on a clean
ROS 2 topic (/gz/iris/pose) so downstream code can consume it without
touching Gazebo's native transport.

The gz-ros bridge (Harmonic/Jazzy) strips model names when converting
gz.msgs.Pose_V -> tf2_msgs/TFMessage — every child_frame_id arrives
empty. So we can't filter by name. Instead we auto-detect the drone by
MOTION: watch all transforms for a few seconds, pick the index that
moved the most. Static scene objects don't move; the drone does once
airborne. After detection we just republish that transform by index.

Pipeline:
  gz sim -> parameter_bridge -> /world/<world>/dynamic_pose/info (TFMessage)
    -> this node -> /gz/iris/pose (PoseStamped)

If detection misbehaves, override with: -p manual_index:=<N>.
Use inspect_gz_transforms.py to figure out which index is the drone.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray


# gz-ros bridge publishes BEST_EFFORT; match it.
GZ_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Our output is RELIABLE so downstream consumers don't miss samples
# during RMSE analysis.
OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class GazeboPoseBridge(Node):

    def __init__(self):
        super().__init__('gz_pose_bridge')

        self.declare_parameter('world_name', 'combined_drone_world',
            ParameterDescriptor(description='Gazebo world name'))
        self.declare_parameter('detect_duration_s', 8.0,
            ParameterDescriptor(description='Auto-detect window length [s]'))
        self.declare_parameter('min_motion_m', 0.5,
            ParameterDescriptor(description='Min motion to qualify as drone [m]'))
        self.declare_parameter('manual_index', -1,
            ParameterDescriptor(description='Skip detection, use this index'))
        self.declare_parameter('diag_rate', 1.0,
            ParameterDescriptor(description='Diagnostics publish rate [Hz]'))

        self.world_name = self.get_parameter('world_name').value
        self.detect_duration = float(self.get_parameter('detect_duration_s').value)
        self.min_motion_m = float(self.get_parameter('min_motion_m').value)
        self.manual_index = int(self.get_parameter('manual_index').value)

        topic_in = f'/world/{self.world_name}/dynamic_pose/info'
        self.sub = self.create_subscription(TFMessage, topic_in, self._cb_tf, GZ_QOS)

        self.pub_pose = self.create_publisher(PoseStamped, '/gz/iris/pose', OUT_QOS)
        self.pub_diag = self.create_publisher(Float64MultiArray, '/gz/iris/bridge_status', 10)

        # Detection state.
        self._first_positions = {}   # idx -> (x0, y0, z0)
        self._max_disp = {}          # idx -> peak displacement
        self._first_msg_time = None
        self._selected_index = None

        if self.manual_index >= 0:
            self._selected_index = self.manual_index
            self.get_logger().info(
                f"[gz_bridge] manual_index={self.manual_index} (skipping detection)")

        self._rx = self._tx = self._skipped = 0

        diag_hz = float(self.get_parameter('diag_rate').value)
        self.create_timer(1.0 / max(diag_hz, 0.1), self._publish_diag)

        self.get_logger().info(
            f"[gz_bridge] up. sub={topic_in} "
            f"detect={self.detect_duration:.0f}s thresh={self.min_motion_m:.1f}m")

    def _cb_tf(self, msg):
        self._rx += 1
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._first_msg_time is None:
            self._first_msg_time = now

        # Steady state: just republish the selected index.
        if self._selected_index is not None:
            idx = self._selected_index
            if idx < 0 or idx >= len(msg.transforms):
                self._skipped += 1
                return
            tf = msg.transforms[idx]
            out = PoseStamped()
            # Fall back to ROS clock if the bridge left stamps at zero.
            if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                out.header.stamp = self.get_clock().now().to_msg()
            else:
                out.header.stamp = tf.header.stamp
            out.header.frame_id = 'world'
            out.pose.position.x = tf.transform.translation.x
            out.pose.position.y = tf.transform.translation.y
            out.pose.position.z = tf.transform.translation.z
            out.pose.orientation.x = tf.transform.rotation.x
            out.pose.orientation.y = tf.transform.rotation.y
            out.pose.orientation.z = tf.transform.rotation.z
            out.pose.orientation.w = tf.transform.rotation.w
            self.pub_pose.publish(out)
            self._tx += 1
            return

        # Detection phase: track per-index displacement.
        for idx, tf in enumerate(msg.transforms):
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z
            if idx not in self._first_positions:
                self._first_positions[idx] = (x, y, z)
                self._max_disp[idx] = 0.0
            else:
                x0, y0, z0 = self._first_positions[idx]
                d = math.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
                if d > self._max_disp[idx]:
                    self._max_disp[idx] = d

        if (now - self._first_msg_time) < self.detect_duration:
            return

        if not self._max_disp:
            return

        best_idx = max(self._max_disp, key=self._max_disp.get)
        best_disp = self._max_disp[best_idx]

        if best_disp >= self.min_motion_m:
            self._selected_index = best_idx
            self.get_logger().info(
                f"[gz_bridge] LOCKED idx={best_idx} moved {best_disp:.2f}m "
                f"(init pos {self._first_positions[best_idx]})")
            top3 = sorted(self._max_disp.items(), key=lambda kv: -kv[1])[:3]
            for i, d in top3:
                self.get_logger().info(f"    idx {i:2d}: {d:.3f}m")
            self.get_logger().info(
                "  if wrong, restart with: -p manual_index:=<N>")
        else:
            # Nothing moved enough yet — the drone probably hasn't taken off.
            # Reset the window and keep learning.
            self.get_logger().warn(
                f"[gz_bridge] no idx moved > {self.min_motion_m:.2f}m in "
                f"{self.detect_duration:.0f}s (max idx {best_idx}={best_disp:.3f}m) "
                f"- waiting for motion",
                throttle_duration_sec=5.0)
            self._first_msg_time = now
            self._first_positions = {}
            self._max_disp = {}

    def _publish_diag(self):
        # Packet: rx, tx, skipped, selected_idx (-1 if not yet).
        msg = Float64MultiArray()
        msg.data = [
            float(self._rx),
            float(self._tx),
            float(self._skipped),
            float(self._selected_index if self._selected_index is not None else -1),
        ]
        self.pub_diag.publish(msg)

        if self._selected_index is None:
            if self._rx == 0:
                self.get_logger().warn(
                    "[gz_bridge] no TF messages yet — is parameter_bridge running?",
                    throttle_duration_sec=5.0)
            else:
                self.get_logger().info(
                    f"[gz_bridge] detecting... rx={self._rx} "
                    f"tracked={len(self._first_positions)}",
                    throttle_duration_sec=5.0)
        else:
            self.get_logger().info(
                f"[gz_bridge] idx={self._selected_index} "
                f"rx={self._rx} tx={self._tx} skipped={self._skipped}",
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
