#!/usr/bin/env python3
"""
Inspect gz transforms.

One-shot diagnostic: subscribes to /world/<world>/dynamic_pose/info,
waits for one message, prints each transform with its index and
position, then exits. Run this after takeoff (drone airborne) to
easily identify which array index is the drone — it'll have the
largest |pos|.

If gz_pose_bridge.py auto-detection picks the wrong index, use this
to find the correct one and then run:
  python3 gz_pose_bridge.py --ros-args -p manual_index:=<N>

  python3 inspect_gz_transforms.py
  python3 inspect_gz_transforms.py --world iris_runway
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from tf2_msgs.msg import TFMessage


GZ_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class Inspector(Node):
    def __init__(self, world):
        super().__init__('gz_inspect')
        topic = f'/world/{world}/dynamic_pose/info'
        self._done = False
        self.sub = self.create_subscription(TFMessage, topic, self._cb, GZ_QOS)
        self.get_logger().info(f'waiting for one message on {topic}...')

    def _cb(self, msg):
        if self._done:
            return
        self._done = True
        print()
        print("=" * 70)
        print(f"  {len(msg.transforms)} transforms in this batch:")
        print("=" * 70)
        print(f"{'idx':>4} | {'x':>10} {'y':>10} {'z':>10} | {'|pos|':>8}")
        print("-" * 70)
        for idx, tf in enumerate(msg.transforms):
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z
            mag = math.sqrt(x * x + y * y + z * z)
            # Hint which index looks like the drone.
            if mag > 5.0:
                marker = "  <-- likely airborne (|pos| > 5 m)"
            elif 0.1 < mag < 1.0:
                marker = "  <-- maybe drone on ground"
            elif mag < 0.01:
                marker = "  <-- likely static origin/world"
            else:
                marker = ""
            print(f"{idx:>4} | {x:>10.3f} {y:>10.3f} {z:>10.3f} | "
                  f"{mag:>8.3f}{marker}")
        print("=" * 70)
        print()
        print("Airborne transforms have the largest |pos| (especially large z).")
        print("Lock gz_pose_bridge to the right index with:")
        print("  python3 gz_pose_bridge.py --ros-args -p manual_index:=<N>")
        print()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='combined_drone_world')
    args, rest = ap.parse_known_args()

    rclpy.init(args=rest)
    node = Inspector(args.world)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
