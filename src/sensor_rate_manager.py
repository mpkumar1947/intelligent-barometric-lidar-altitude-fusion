#!/usr/bin/env python3
"""
Sensor rate manager.

Decouples raw sensor publish rates from fusion node consumption rates and
optionally injects synthetic disturbances for stress testing. Rates and
disturbances are tunable at runtime via ros2 param set.

Disturbance modes: NONE, WIND, LIDAR_DROPOUT, BARO_DRIFT, COMBINED.
A mode is only applied when disturbance_active is also True (two-tier
arming so scenario scripts can configure mode before takeoff and flip
active once airborne).

Diagnostics topic: /sensor_rate_manager/diagnostics
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from sensor_msgs.msg import Imu, FluidPressure, LaserScan
from std_msgs.msg import Float64MultiArray


# Match MAVROS best-effort QoS on the input side.
MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Reliable on the output side so downstream nodes never miss a sample.
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


MODE_NONE = 'NONE'
MODE_WIND = 'WIND'
MODE_LIDAR_DROPOUT = 'LIDAR_DROPOUT'
MODE_BARO_DRIFT = 'BARO_DRIFT'
MODE_COMBINED = 'COMBINED'

VALID_MODES = (MODE_NONE, MODE_WIND, MODE_LIDAR_DROPOUT,
               MODE_BARO_DRIFT, MODE_COMBINED)

# Numeric encoding for the diagnostics packet.
MODE_ENUM = {
    MODE_NONE: 0,
    MODE_WIND: 1,
    MODE_LIDAR_DROPOUT: 2,
    MODE_BARO_DRIFT: 3,
    MODE_COMBINED: 4,
}


class SensorRateManager(Node):

    def __init__(self):
        super().__init__('sensor_rate_manager')

        # Rate parameters.
        self.declare_parameter('imu_rate', 100.0,
            ParameterDescriptor(description='IMU republish rate [Hz]'))
        self.declare_parameter('baro_rate', 4.0,
            ParameterDescriptor(description='Baro republish rate [Hz]'))
        self.declare_parameter('lidar_rate', 99.0,
            ParameterDescriptor(description='LiDAR republish rate [Hz]'))
        self.declare_parameter('diagnostics_rate', 1.0,
            ParameterDescriptor(description='Diagnostics publish rate [Hz]'))

        # Disturbance master switches.
        self.declare_parameter('disturbance_mode', MODE_NONE,
            ParameterDescriptor(description=f'One of {VALID_MODES}'))
        self.declare_parameter('disturbance_active', False,
            ParameterDescriptor(description='Master enable (arm/disarm)'))
        self.declare_parameter('disturbance_seed', 42,
            ParameterDescriptor(description='RNG seed for reproducibility'))

        # Per-mode tunables. Defaults tuned empirically so a 9-minute mission
        # produces visible disturbance without triggering the failsafe prematurely.
        self.declare_parameter('wind_baro_rw_sigma_pa', 8.0)
        self.declare_parameter('wind_baro_gauss_sigma_pa', 15.0)
        self.declare_parameter('wind_baro_rw_decay', 0.995)

        self.declare_parameter('lidar_dropout_burst_prob', 0.02)
        self.declare_parameter('lidar_dropout_burst_len', 25)
        self.declare_parameter('lidar_spike_prob', 0.005)
        self.declare_parameter('lidar_spike_magnitude_m', 30.0)

        self.declare_parameter('baro_drift_rate_pa_per_s', 5.0)
        self.declare_parameter('baro_drift_oscillation_hz', 0.02)
        self.declare_parameter('baro_drift_osc_amp_pa', 40.0)

        self.add_on_set_parameters_callback(self._on_param_change)

        # Latest-sample caches.
        self._latest_imu: Imu = None
        self._latest_baro: FluidPressure = None
        self._latest_lidar: LaserScan = None

        # Counters for diagnostics.
        self._rx_imu = self._rx_baro = self._rx_lidar = 0
        self._tx_imu = self._tx_baro = self._tx_lidar = 0
        self._drop_imu = self._drop_baro = self._drop_lidar = 0

        # Disturbance state.
        self._rng = random.Random(self.get_parameter('disturbance_seed').value)
        self._wind_rw_state = 0.0
        self._drift_t0 = None
        self._last_baro_delta_pa = 0.0
        self._lidar_in_burst = False
        self._lidar_burst_remain = 0
        self._lidar_burst_count = 0

        self.create_subscription(
            Imu, '/mavros/imu/data', self._cb_imu, MAVROS_QOS)
        self.create_subscription(
            FluidPressure, '/mavros/imu/static_pressure', self._cb_baro, MAVROS_QOS)
        self.create_subscription(
            LaserScan, '/lidar_down', self._cb_lidar, MAVROS_QOS)

        self.pub_imu = self.create_publisher(
            Imu, '/managed/imu/data', RELIABLE_QOS)
        self.pub_baro = self.create_publisher(
            FluidPressure, '/managed/imu/static_pressure', RELIABLE_QOS)
        self.pub_lidar = self.create_publisher(
            LaserScan, '/managed/lidar_down', RELIABLE_QOS)
        self.pub_diag = self.create_publisher(
            Float64MultiArray, '/sensor_rate_manager/diagnostics', 10)

        self._timer_imu = self._make_timer('imu_rate', self._publish_imu)
        self._timer_baro = self._make_timer('baro_rate', self._publish_baro)
        self._timer_lidar = self._make_timer('lidar_rate', self._publish_lidar)
        self._timer_diag = self._make_timer('diagnostics_rate', self._publish_diag)

        self.get_logger().info(
            f"[rate_manager] up. "
            f"imu={self.get_parameter('imu_rate').value} Hz "
            f"baro={self.get_parameter('baro_rate').value} Hz "
            f"lidar={self.get_parameter('lidar_rate').value} Hz "
            f"dist={self.get_parameter('disturbance_mode').value}"
            f"/{self.get_parameter('disturbance_active').value}"
        )

    def _make_timer(self, param_name, callback):
        rate = self.get_parameter(param_name).value
        return self.create_timer(1.0 / rate, callback)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- raw sensor intake ------------------------------------------------

    def _cb_imu(self, msg):
        self._latest_imu = msg
        self._rx_imu += 1

    def _cb_baro(self, msg):
        self._latest_baro = msg
        self._rx_baro += 1

    def _cb_lidar(self, msg):
        self._latest_lidar = msg
        self._rx_lidar += 1

    # --- disturbance helpers ----------------------------------------------

    def _disturbance_active(self):
        mode = self.get_parameter('disturbance_mode').value
        active = self.get_parameter('disturbance_active').value
        return (mode != MODE_NONE) and bool(active)

    def _mode_affects_baro(self):
        m = self.get_parameter('disturbance_mode').value
        return m in (MODE_WIND, MODE_BARO_DRIFT, MODE_COMBINED)

    def _mode_affects_lidar(self):
        m = self.get_parameter('disturbance_mode').value
        return m in (MODE_LIDAR_DROPOUT, MODE_COMBINED)

    def _reset_disturbance_state(self):
        # Called on any mode/active/seed change so each arm sees a fresh start.
        self._rng = random.Random(self.get_parameter('disturbance_seed').value)
        self._wind_rw_state = 0.0
        self._drift_t0 = None
        self._last_baro_delta_pa = 0.0
        self._lidar_in_burst = False
        self._lidar_burst_remain = 0
        self.get_logger().info('[rate_manager] disturbance state reset')

    def _disturb_baro_pressure(self, P_in):
        if not self._disturbance_active() or not self._mode_affects_baro():
            self._last_baro_delta_pa = 0.0
            return P_in

        delta = 0.0
        mode = self.get_parameter('disturbance_mode').value

        if mode in (MODE_WIND, MODE_COMBINED):
            # Random walk + white noise on pressure. The RW captures slow
            # gust correlations; the Gaussian captures fast turbulence.
            rw_sig = float(self.get_parameter('wind_baro_rw_sigma_pa').value)
            gauss_sig = float(self.get_parameter('wind_baro_gauss_sigma_pa').value)
            decay = float(self.get_parameter('wind_baro_rw_decay').value)
            self._wind_rw_state = (self._wind_rw_state * decay
                                   + self._rng.gauss(0.0, rw_sig))
            delta += self._wind_rw_state + self._rng.gauss(0.0, gauss_sig)

        if mode in (MODE_BARO_DRIFT, MODE_COMBINED):
            # Linear drift + slow oscillation (simulates temperature / cal drift).
            if self._drift_t0 is None:
                self._drift_t0 = self._now()
            t = self._now() - self._drift_t0
            rate = float(self.get_parameter('baro_drift_rate_pa_per_s').value)
            osc_hz = float(self.get_parameter('baro_drift_oscillation_hz').value)
            osc_a = float(self.get_parameter('baro_drift_osc_amp_pa').value)
            delta += rate * t + osc_a * math.sin(2.0 * math.pi * osc_hz * t)

        self._last_baro_delta_pa = delta
        return P_in + delta

    def _disturb_lidar_range(self, r_in):
        if not self._disturbance_active() or not self._mode_affects_lidar():
            return r_in

        # Burst dropout (simulates out-of-range / occluded).
        if self._lidar_in_burst:
            self._lidar_burst_remain -= 1
            if self._lidar_burst_remain <= 0:
                self._lidar_in_burst = False
            return float('inf')

        burst_prob = float(self.get_parameter('lidar_dropout_burst_prob').value)
        if self._rng.random() < burst_prob:
            self._lidar_in_burst = True
            self._lidar_burst_remain = int(self.get_parameter('lidar_dropout_burst_len').value)
            self._lidar_burst_count += 1
            return float('inf')

        # Single-sample spike (simulates glint / spurious return).
        spike_prob = float(self.get_parameter('lidar_spike_prob').value)
        if self._rng.random() < spike_prob:
            mag = float(self.get_parameter('lidar_spike_magnitude_m').value)
            sign = 1.0 if self._rng.random() > 0.5 else -1.0
            return max(0.1, r_in + sign * mag)

        return r_in

    # --- timer callbacks --------------------------------------------------

    def _publish_imu(self):
        # IMU is never disturbed — pass-through only.
        if self._latest_imu is not None:
            self.pub_imu.publish(self._latest_imu)
            self._tx_imu += 1
        else:
            self._drop_imu += 1

    def _publish_baro(self):
        if self._latest_baro is None:
            self._drop_baro += 1
            return

        P_raw = float(self._latest_baro.fluid_pressure)
        P_out = self._disturb_baro_pressure(P_raw)

        # Build a fresh message — never mutate the cached original.
        out = FluidPressure()
        out.header = self._latest_baro.header
        out.fluid_pressure = P_out
        out.variance = self._latest_baro.variance
        self.pub_baro.publish(out)
        self._tx_baro += 1

    def _publish_lidar(self):
        if self._latest_lidar is None:
            self._drop_lidar += 1
            return

        src = self._latest_lidar

        if (self._disturbance_active()
                and self._mode_affects_lidar()
                and src.ranges):
            r_out = self._disturb_lidar_range(float(src.ranges[0]))
            out = LaserScan()
            out.header = src.header
            out.angle_min = src.angle_min
            out.angle_max = src.angle_max
            out.angle_increment = src.angle_increment
            out.time_increment = src.time_increment
            out.scan_time = src.scan_time
            out.range_min = src.range_min
            out.range_max = src.range_max
            out.ranges = [r_out] + list(src.ranges[1:])
            out.intensities = list(src.intensities)
            self.pub_lidar.publish(out)
        else:
            self.pub_lidar.publish(src)

        self._tx_lidar += 1

    # --- diagnostics ------------------------------------------------------

    def _publish_diag(self):
        # Packet layout (16 fields):
        #   0..2   imu/baro/lidar rate params
        #   3..5   rx counts
        #   6..8   tx counts
        #   9..11  drop counts
        #   12     disturbance mode enum
        #   13     last baro Δp in Pa
        #   14     1 if LiDAR currently in burst else 0
        #   15     cumulative burst count
        mode = self.get_parameter('disturbance_mode').value
        msg = Float64MultiArray()
        msg.data = [
            self.get_parameter('imu_rate').value,
            self.get_parameter('baro_rate').value,
            self.get_parameter('lidar_rate').value,
            float(self._rx_imu), float(self._rx_baro), float(self._rx_lidar),
            float(self._tx_imu), float(self._tx_baro), float(self._tx_lidar),
            float(self._drop_imu), float(self._drop_baro), float(self._drop_lidar),
            float(MODE_ENUM.get(mode, 0)),
            float(self._last_baro_delta_pa),
            1.0 if self._lidar_in_burst else 0.0,
            float(self._lidar_burst_count),
        ]
        self.pub_diag.publish(msg)

        state = 'ACTIVE' if self._disturbance_active() else 'idle'
        self.get_logger().info(
            f"[rate_manager] mode={mode}/{state} "
            f"imu={self._rx_imu}/{self._tx_imu} "
            f"baro={self._rx_baro}/{self._tx_baro} Δp={self._last_baro_delta_pa:+.1f}Pa "
            f"lidar={self._rx_lidar}/{self._tx_lidar} bursts={self._lidar_burst_count}"
        )

    # --- runtime param callback -------------------------------------------

    def _on_param_change(self, params):
        # Rebuild the timer when a rate changes so the new period takes
        # effect immediately. On mode/active/seed changes, wipe the
        # accumulator state.
        timer_map = {
            'imu_rate': ('_timer_imu', self._publish_imu),
            'baro_rate': ('_timer_baro', self._publish_baro),
            'lidar_rate': ('_timer_lidar', self._publish_lidar),
            'diagnostics_rate': ('_timer_diag', self._publish_diag),
        }

        for p in params:
            if p.name in timer_map and p.value > 0.0:
                attr, cb = timer_map[p.name]
                getattr(self, attr).cancel()
                setattr(self, attr, self.create_timer(1.0 / float(p.value), cb))
                self.get_logger().info(
                    f"[rate_manager] {p.name} -> {p.value} Hz")

            elif p.name == 'disturbance_mode':
                if p.value not in VALID_MODES:
                    self.get_logger().warn(
                        f"[rate_manager] invalid mode '{p.value}'")
                    return SetParametersResult(
                        successful=False,
                        reason=f"invalid mode; use one of {VALID_MODES}")
                self._reset_disturbance_state()
                self.get_logger().info(
                    f"[rate_manager] mode -> {p.value}")

            elif p.name == 'disturbance_active':
                self._reset_disturbance_state()
                self.get_logger().info(
                    f"[rate_manager] {'ACTIVE' if p.value else 'idle'}")

            elif p.name == 'disturbance_seed':
                self._reset_disturbance_state()
                self.get_logger().info(
                    f"[rate_manager] seed -> {p.value}")

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SensorRateManager()
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
