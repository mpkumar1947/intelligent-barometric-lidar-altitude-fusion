#!/usr/bin/env python3
"""
Failsafe monitor.

Three independent failure domains in one node:
  1. LiDAR integrity (three-tier escalation: degraded -> failed -> landing)
  2. Battery monitor (triggers retrospective landing search below threshold)
  3. Variance ring buffer (1 Hz samples on flat legs, used for landing-target scoring)

This node never writes directly to /mavros/setpoint_raw/local. When the mission
should hand control over, we publish to /failsafe/command and the mission node
forwards those messages. That keeps the MAVROS write path single-owner.

Manual trigger for demos:
  ros2 param set /failsafe_monitor force_state_override 3
      0=NOMINAL 1=DEGRADED 2=FAILED 3=BATTERY_LOW 4=LANDING_COMMITTED 5=LANDED
  -1 disables the override.
"""

import math
import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan, Imu, BatteryState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Float64MultiArray, Int8
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode


MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# Must match mission_node.FS exactly (both sides read/write /failsafe/state).
class FS:
    NOMINAL           = 0
    LIDAR_DEGRADED    = 1
    LIDAR_FAILED      = 2
    BATTERY_LOW       = 3
    LANDING_COMMITTED = 4
    LANDED            = 5


_STATE_NAME = {
    FS.NOMINAL:           'NOMINAL',
    FS.LIDAR_DEGRADED:    'LIDAR_DEGRADED',
    FS.LIDAR_FAILED:      'LIDAR_FAILED',
    FS.BATTERY_LOW:       'BATTERY_LOW',
    FS.LANDING_COMMITTED: 'LANDING_COMMITTED',
    FS.LANDED:            'LANDED',
}

# Must match mission_node.LegType. Variance buffer only samples during
# TRANSIT / ORBIT so the buffer contains steady-state readings.
LEG_CLIMB   = 0
LEG_TRANSIT = 1
LEG_ORBIT   = 2
LEG_LAND    = 3


# Compass convention: 0° = North, 90° = East, clockwise.
# Duplicated from mission_node to keep the two nodes decoupled.
def heading_to_enu_velocity(speed, heading_deg):
    h = math.radians(heading_deg)
    return speed * math.sin(h), speed * math.cos(h)


def heading_to_enu_yaw(heading_deg):
    return math.radians(90.0 - heading_deg)


def bearing_to_target(x_from, y_from, x_to, y_to):
    # atan2(dx_east, dy_north) — note the argument order gives compass bearing.
    return math.degrees(math.atan2(x_to - x_from, y_to - y_from)) % 360.0


# PositionTarget builder. Same mask policy as mission_node: we command
# horizontal velocity, vertical position, and absolute yaw.
IGNORE_PX, IGNORE_PY, IGNORE_PZ     = 1, 2, 4
IGNORE_VX, IGNORE_VY, IGNORE_VZ     = 8, 16, 32
IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ  = 64, 128, 256
IGNORE_YAW, IGNORE_YAW_RATE         = 1024, 2048

MASK_VEL_XY_POS_Z_YAW = (IGNORE_PX | IGNORE_PY | IGNORE_VZ
                         | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
                         | IGNORE_YAW_RATE)


def build_setpoint(stamp, target_alt, vx_east, vy_north, heading_deg):
    m = PositionTarget()
    m.header.stamp = stamp
    m.header.frame_id = 'map'
    m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    m.type_mask = MASK_VEL_XY_POS_Z_YAW
    m.position.z = float(target_alt)
    m.velocity.x = float(vx_east)
    m.velocity.y = float(vy_north)
    m.yaw = float(heading_to_enu_yaw(heading_deg))
    return m


class FailsafeNode(Node):

    def __init__(self):
        super().__init__('failsafe_monitor')

        # LiDAR integrity thresholds.
        self.declare_parameter('lidar_invalid_min_duration_s', 0.5)
        self.declare_parameter('lidar_degraded_to_failed_s',   3.0)
        self.declare_parameter('lidar_failed_to_landing_s',   15.0)
        self.declare_parameter('lidar_recovery_hold_s',        2.0)
        self.declare_parameter('lidar_range_min_m', 0.1)
        self.declare_parameter('lidar_range_max_m', 180.0)

        self.declare_parameter('battery_crit_pct', 25.0)

        # Variance buffer config.
        self.declare_parameter('variance_flat_threshold',   0.05)
        self.declare_parameter('variance_window_samples',   30)
        self.declare_parameter('variance_sample_interval_s', 1.0)
        self.declare_parameter('variance_buffer_max_samples', 600)

        # Landing-search config.
        self.declare_parameter('landing_search_max_age_s', 300.0)
        self.declare_parameter('landing_cruise_alt_m',      40.0)
        self.declare_parameter('landing_cruise_speed_mps',   3.0)
        self.declare_parameter('landing_arrival_radius_m',   3.0)
        self.declare_parameter('landing_hover_verify_s',     5.0)
        self.declare_parameter('landing_final_hover_alt_m',  5.0)
        self.declare_parameter('landing_descent_rate_mps',   0.5)
        self.declare_parameter('landing_max_retries', 2)

        # Retreat sub-phase (during LIDAR_FAILED).
        self.declare_parameter('retreat_yaw_reverse_s', 2.0)
        self.declare_parameter('retreat_duration_s',    5.0)
        self.declare_parameter('retreat_speed_mps',     2.0)

        # -1 disables, 0..5 forces that state. SITL doesn't drain battery
        # naturally, so this is how we demo BATTERY_LOW.
        self.declare_parameter('force_state_override', -1)

        # --- state ---
        self.state = FS.NOMINAL
        self.state_t0 = self._now()

        # LiDAR integrity timers.
        self.lidar_invalid_t0 = None
        self.lidar_valid_t0 = None
        self.last_lidar_range = None
        self.range_window = collections.deque(maxlen=30)

        self.battery_pct = 100.0

        # Position estimate (mirrors mission_node dead-reckoning when mission is
        # driving; we track it ourselves while we have override authority).
        self.x_est = 0.0
        self.y_est = 0.0
        self.last_dr_t = None
        self.fused_alt = 0.0

        # Leg context — we only sample variance during TRANSIT/ORBIT on target.
        self.current_leg_type = None
        self.current_leg_ontarget = False
        self.current_leg_target_alt = None

        # Buffered samples for retrospective landing-target selection.
        self.variance_buffer = collections.deque(
            maxlen=self.get_parameter('variance_buffer_max_samples').value)
        self.last_sample_t = None

        # Landing search state.
        self.landing_target = None
        self.landing_blocklist = []
        self.landing_retries = 0
        self.landing_phase = None
        self.landing_phase_t0 = None
        self.verify_var_samples = []

        # Retreat sub-phase state.
        self.retreat_phase = None
        self.retreat_phase_t0 = None
        self.retreat_heading = 0.0

        # Only populated while we're publishing an override.
        self.current_override = None

        # --- subscriptions ---
        self.create_subscription(LaserScan, '/managed/lidar_down',
                                 self._cb_lidar, MAVROS_QOS)
        self.create_subscription(Float64MultiArray, '/fusion/diagnostics',
                                 self._cb_diag, MAVROS_QOS)
        self.create_subscription(BatteryState, '/mavros/battery',
                                 self._cb_battery, MAVROS_QOS)
        self.create_subscription(PoseStamped, '/mission/dead_reckoned_pose',
                                 self._cb_dr_pose, 10)
        self.create_subscription(Float64MultiArray, '/mission/leg_info',
                                 self._cb_leg_info, RELIABLE_QOS)
        self.create_subscription(Float64, '/fusion/stage2b/ekf2_altitude',
                                 lambda m: setattr(self, 'fused_alt', m.data),
                                 MAVROS_QOS)

        self.pub_state = self.create_publisher(Int8, '/failsafe/state', RELIABLE_QOS)
        self.pub_cmd = self.create_publisher(PositionTarget, '/failsafe/command', RELIABLE_QOS)
        self.pub_diag = self.create_publisher(Float64MultiArray, '/failsafe/diagnostics', 10)

        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')

        self.add_on_set_parameters_callback(self._on_param_change)

        # 20 Hz override stream, 10 Hz FSM tick, 2 Hz state + diag.
        self.create_timer(0.05, self._cb_override_pub)
        self.create_timer(0.1,  self._cb_fsm)
        self.create_timer(0.5,  self._cb_state_pub)
        self.create_timer(0.5,  self._cb_diag_pub)

        self.get_logger().info(
            "[failsafe] up. override=/failsafe/command (20Hz) state=/failsafe/state")

    # --- clock / state helpers --------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, new_state):
        if new_state == self.state:
            return
        self.get_logger().warn(
            f'[failsafe] {_STATE_NAME[self.state]} -> {_STATE_NAME[new_state]}')
        self.state = new_state
        self.state_t0 = self._now()

    # --- subscribers ------------------------------------------------------

    def _cb_lidar(self, msg):
        r_min = self.get_parameter('lidar_range_min_m').value
        r_max = self.get_parameter('lidar_range_max_m').value

        r = msg.ranges[0] if msg.ranges else float('inf')
        invalid = (not math.isfinite(r)) or r < r_min or r > r_max
        now = self._now()

        # Update invalidity timers.
        if invalid:
            if self.lidar_invalid_t0 is None:
                self.lidar_invalid_t0 = now
            self.lidar_valid_t0 = None
        else:
            if self.lidar_valid_t0 is None:
                self.lidar_valid_t0 = now
            self.last_lidar_range = r
            self.range_window.append(r)

        # Variance-buffer sampling — only on valid TRANSIT/ORBIT on-target.
        if invalid:
            return
        if self.current_leg_type not in (LEG_TRANSIT, LEG_ORBIT):
            return
        if not self.current_leg_ontarget:
            return

        interval = self.get_parameter('variance_sample_interval_s').value
        if self.last_sample_t is not None and (now - self.last_sample_t) < interval:
            return
        if len(self.range_window) < 5:
            return

        win = list(self.range_window)
        self.variance_buffer.append({
            'x':    self.x_est,
            'y':    self.y_est,
            'var':  float(_variance(win)),
            'mean': float(sum(win) / len(win)),
            't':    now,
        })
        self.last_sample_t = now

    def _cb_diag(self, msg):
        # Reserved — EKF-2 σ could become a trust signal later.
        pass

    def _cb_battery(self, msg):
        # MAVROS sometimes reports percentage in [0,1], sometimes [0,100].
        # Detect and normalise.
        if msg.percentage > 1.5:
            self.battery_pct = float(msg.percentage)
        else:
            self.battery_pct = float(msg.percentage) * 100.0

    def _cb_dr_pose(self, msg):
        # While the mission is driving, mirror its position. While we're
        # driving (LIDAR_FAILED or later), we integrate ourselves.
        if self.state < FS.LIDAR_FAILED:
            self.x_est = msg.pose.position.x
            self.y_est = msg.pose.position.y
        self.last_dr_t = self._now()

    def _cb_leg_info(self, msg):
        if len(msg.data) < 7:
            return
        self.current_leg_type       = int(msg.data[1])
        self.current_leg_target_alt = float(msg.data[3])
        self.current_leg_ontarget = (
            abs(self.fused_alt - self.current_leg_target_alt) < 3.0)

    # --- runtime param override -------------------------------------------

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'force_state_override':
                val = int(p.value)
                if 0 <= val <= FS.LANDED:
                    self.get_logger().warn(
                        f'[failsafe] manual override -> {_STATE_NAME[val]}')
                    self._force_transition_to(val)
        return SetParametersResult(successful=True)

    def _force_transition_to(self, new_state):
        if new_state == FS.BATTERY_LOW:
            self._set_state(FS.BATTERY_LOW)
            self._begin_landing_search()
        elif new_state == FS.LIDAR_FAILED:
            self._set_state(FS.LIDAR_FAILED)
            self._begin_retreat()
        else:
            self._set_state(new_state)

    # --- main FSM (10 Hz) -------------------------------------------------

    def _cb_fsm(self):
        now = self._now()

        force = self.get_parameter('force_state_override').value
        if force >= 0 and int(force) != self.state and int(force) <= FS.LANDED:
            self._force_transition_to(int(force))
            return

        # Battery preempts any non-landing state.
        batt_crit = self.get_parameter('battery_crit_pct').value
        if (self.battery_pct < batt_crit
                and self.state not in (FS.BATTERY_LOW,
                                       FS.LANDING_COMMITTED,
                                       FS.LANDED)):
            self.get_logger().warn(
                f'[failsafe] battery {self.battery_pct:.1f}% < {batt_crit:.1f}% '
                '-> landing search')
            self._set_state(FS.BATTERY_LOW)
            self._begin_landing_search()
            return

        # LiDAR integrity.
        t_min  = self.get_parameter('lidar_invalid_min_duration_s').value
        t_fail = self.get_parameter('lidar_degraded_to_failed_s').value
        t_land = self.get_parameter('lidar_failed_to_landing_s').value
        t_rec  = self.get_parameter('lidar_recovery_hold_s').value

        invalid_dur = ((now - self.lidar_invalid_t0)
                       if self.lidar_invalid_t0 is not None else 0.0)
        valid_dur   = ((now - self.lidar_valid_t0)
                       if self.lidar_valid_t0 is not None else 0.0)

        if self.state == FS.NOMINAL:
            if invalid_dur >= t_fail:
                self._set_state(FS.LIDAR_FAILED)
                self._begin_retreat()
            elif invalid_dur >= t_min:
                self._set_state(FS.LIDAR_DEGRADED)

        elif self.state == FS.LIDAR_DEGRADED:
            if invalid_dur >= t_fail:
                self._set_state(FS.LIDAR_FAILED)
                self._begin_retreat()
            elif valid_dur >= t_rec:
                self._set_state(FS.NOMINAL)

        elif self.state == FS.LIDAR_FAILED:
            self._tick_retreat(now)
            if invalid_dur >= t_land:
                self.get_logger().warn(
                    f'[failsafe] lidar failed >{t_land:.0f}s -> landing')
                self._set_state(FS.BATTERY_LOW)
                self._begin_landing_search()
                return
            if valid_dur >= t_rec:
                self._set_state(FS.NOMINAL)
                self.current_override = None

        elif self.state == FS.BATTERY_LOW:
            # Search already initiated in _begin_landing_search; nothing to do
            # here until _tick_landing runs next tick.
            pass

        elif self.state == FS.LANDING_COMMITTED:
            self._tick_landing(now)

        elif self.state == FS.LANDED:
            self.current_override = None

    # --- retreat sub-state (during LIDAR_FAILED) --------------------------
    # HOVER (2s) -> FLY_BACK (5s @ reversed heading) -> HOLD until recovery

    def _begin_retreat(self):
        self.retreat_phase = 'HOVER'
        self.retreat_phase_t0 = self._now()
        # We don't have immediate access to the previous leg's heading here,
        # so HOVER + HOLD is the main strategy. 0° is fine as a default since
        # the retreat distance is small and HOLD is where recovery happens.
        self.retreat_heading = 0.0
        self.get_logger().warn('[failsafe] retreat: HOVER 2s')

    def _tick_retreat(self, now):
        hover_s = self.get_parameter('retreat_yaw_reverse_s').value
        fly_s   = self.get_parameter('retreat_duration_s').value
        speed   = self.get_parameter('retreat_speed_mps').value

        elapsed = now - (self.retreat_phase_t0 or now)
        alt = self.current_leg_target_alt if self.current_leg_target_alt else self.fused_alt

        if self.retreat_phase == 'HOVER':
            self.current_override = build_setpoint(
                self.get_clock().now().to_msg(),
                alt, 0.0, 0.0, self.retreat_heading)
            if elapsed >= hover_s:
                self.retreat_heading = (self.retreat_heading + 180.0) % 360.0
                self.retreat_phase = 'FLY_BACK'
                self.retreat_phase_t0 = now
                self.get_logger().warn(
                    f'[failsafe] retreat: FLY_BACK hdg={self.retreat_heading:.0f}° '
                    f'{fly_s:.0f}s @ {speed:.1f}m/s')

        elif self.retreat_phase == 'FLY_BACK':
            vx, vy = heading_to_enu_velocity(speed, self.retreat_heading)
            self.current_override = build_setpoint(
                self.get_clock().now().to_msg(),
                alt, vx, vy, self.retreat_heading)
            # Mission isn't publishing DR any more — integrate here.
            self._advance_self_position(speed, self.retreat_heading)

            if elapsed >= fly_s:
                self.retreat_phase = 'HOLD'
                self.retreat_phase_t0 = now
                self.get_logger().warn('[failsafe] retreat: HOLD')

        elif self.retreat_phase == 'HOLD':
            self.current_override = build_setpoint(
                self.get_clock().now().to_msg(),
                alt, 0.0, 0.0, self.retreat_heading)

    # --- retrospective landing search -------------------------------------

    def _begin_landing_search(self):
        target = self._select_landing_target()
        if target is None:
            self.get_logger().warn(
                '[failsafe] no landing candidate — falling back to (0,0)')
            target = {'x': 0.0, 'y': 0.0, 'var': 0.0,
                      'mean': 10.0, 't': self._now(), '_fallback': True}
        else:
            self.get_logger().info(
                f'[failsafe] landing target=({target["x"]:.1f},{target["y"]:.1f}) '
                f'var={target["var"]:.3f} age={self._now() - target["t"]:.0f}s')
        self.landing_target = target
        self.landing_phase = 'NAV'
        self.landing_phase_t0 = self._now()
        self.verify_var_samples = []
        self._set_state(FS.LANDING_COMMITTED)

    def _select_landing_target(self):
        # Score by variance; tiebreak with age (prefer recent). Reject blocked,
        # too-old, and out-of-band samples.
        var_thresh = self.get_parameter('variance_flat_threshold').value
        max_age = self.get_parameter('landing_search_max_age_s').value
        now = self._now()

        candidates = []
        for i, e in enumerate(self.variance_buffer):
            if i in self.landing_blocklist:
                continue
            age = now - e['t']
            if age > max_age:
                continue
            if e['mean'] < 2.0 or e['mean'] > 50.0:
                continue
            if e['var'] > var_thresh:
                continue
            candidates.append((e['var'] + 0.001 * age, i, e))

        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0])
        return candidates[0][2]

    def _tick_landing(self, now):
        phase = self.landing_phase
        target = self.landing_target
        cruise_alt   = self.get_parameter('landing_cruise_alt_m').value
        cruise_speed = self.get_parameter('landing_cruise_speed_mps').value
        arr_r        = self.get_parameter('landing_arrival_radius_m').value
        hover_s      = self.get_parameter('landing_hover_verify_s').value
        final_alt    = self.get_parameter('landing_final_hover_alt_m').value
        descent_v    = self.get_parameter('landing_descent_rate_mps').value

        if phase == 'NAV':
            dist = math.hypot(target['x'] - self.x_est,
                              target['y'] - self.y_est)
            if dist < arr_r:
                self.get_logger().info(f'[failsafe] arrived ({dist:.1f}m) -> VERIFY')
                self.landing_phase = 'VERIFY'
                self.landing_phase_t0 = now
                self.verify_var_samples = []
                return

            brng = bearing_to_target(self.x_est, self.y_est,
                                     target['x'], target['y'])
            vx, vy = heading_to_enu_velocity(cruise_speed, brng)
            self.current_override = build_setpoint(
                self.get_clock().now().to_msg(),
                cruise_alt, vx, vy, brng)
            self._advance_self_position(cruise_speed, brng)

        elif phase == 'VERIFY':
            # Hover and re-sample ground flatness.
            self.current_override = build_setpoint(
                self.get_clock().now().to_msg(),
                cruise_alt, 0.0, 0.0, 0.0)
            if self.last_lidar_range is not None:
                self.verify_var_samples.append(self.last_lidar_range)

            elapsed = now - (self.landing_phase_t0 or now)
            if elapsed >= hover_s:
                if len(self.verify_var_samples) < 5:
                    self.get_logger().warn('[failsafe] verify: not enough samples, proceeding')
                    self.landing_phase = 'DESCEND'
                    self.landing_phase_t0 = now
                    return
                v = _variance(self.verify_var_samples)
                thresh_2x = 2.0 * self.get_parameter('variance_flat_threshold').value
                if v > thresh_2x:
                    self.get_logger().warn(
                        f'[failsafe] verify FAIL var={v:.3f} > {thresh_2x:.3f}')
                    if self.landing_retries < self.get_parameter('landing_max_retries').value:
                        self.landing_retries += 1
                        # Blocklist this candidate (unless it's the fallback).
                        if not self.landing_target.get('_fallback', False):
                            for i, e in enumerate(self.variance_buffer):
                                if (e['x'] == self.landing_target['x']
                                        and e['y'] == self.landing_target['y']
                                        and e['t'] == self.landing_target['t']):
                                    self.landing_blocklist.append(i)
                                    break
                        self._begin_landing_search()
                        return
                    else:
                        self.get_logger().warn('[failsafe] max retries — forcing descent')
                        self.landing_phase = 'DESCEND'
                        self.landing_phase_t0 = now
                else:
                    self.get_logger().info(
                        f'[failsafe] verify OK var={v:.3f} -> DESCEND')
                    self.landing_phase = 'DESCEND'
                    self.landing_phase_t0 = now

        elif phase == 'DESCEND':
            # Linear descent to final_alt, then hand off to ArduCopter LAND mode.
            elapsed = now - (self.landing_phase_t0 or now)
            t_to_final = max(cruise_alt - final_alt, 0.1) / descent_v
            cur_target_alt = (cruise_alt - descent_v * elapsed
                              if elapsed < t_to_final else final_alt)

            self.current_override = build_setpoint(
                self.get_clock().now().to_msg(),
                cur_target_alt, 0.0, 0.0, 0.0)

            if elapsed > t_to_final + 1.0:
                self.get_logger().info('[failsafe] handoff -> LAND mode')
                req = SetMode.Request()
                req.custom_mode = 'LAND'
                self.cli_mode.call_async(req)
                self.landing_phase = 'HANDED_OFF'
                self.landing_phase_t0 = now

        elif phase == 'HANDED_OFF':
            # ArduCopter LAND now owns the drone. We keep publishing a safe
            # hover in case the mode switch hasn't taken effect yet.
            self.current_override = build_setpoint(
                self.get_clock().now().to_msg(),
                self.get_parameter('landing_final_hover_alt_m').value,
                0.0, 0.0, 0.0)
            elapsed = now - (self.landing_phase_t0 or now)
            if elapsed > 30.0:
                self.get_logger().info('[failsafe] LAND timeout -> LANDED')
                self._set_state(FS.LANDED)
                self.current_override = None

    # --- self-position integration (during override) ----------------------

    def _advance_self_position(self, speed, heading_deg):
        now = self._now()
        if self.last_dr_t is None:
            self.last_dr_t = now
            return
        dt = now - self.last_dr_t
        self.last_dr_t = now
        if speed > 0.0 and dt < 0.5:
            vx, vy = heading_to_enu_velocity(speed, heading_deg)
            self.x_est += vx * dt
            self.y_est += vy * dt

    # --- publishers -------------------------------------------------------

    def _cb_override_pub(self):
        # Only publish while we have authority.
        if self.state < FS.LIDAR_FAILED:
            return
        if self.current_override is None:
            return
        self.pub_cmd.publish(self.current_override)

    def _cb_state_pub(self):
        msg = Int8()
        msg.data = int(self.state)
        self.pub_state.publish(msg)

    def _cb_diag_pub(self):
        # Packet layout:
        #   0 state    1 invalid_dur   2 battery_pct   3 buffer_size
        #   4..6 best candidate (x, y, var)    7 best_age
        #   8..9 landing_target x, y  (NaN unless LANDING_COMMITTED)
        invalid_dur = ((self._now() - self.lidar_invalid_t0)
                       if self.lidar_invalid_t0 is not None else 0.0)
        best = self.landing_target or {}
        best_age = (self._now() - best['t']) if 't' in best else float('nan')
        tx = best.get('x', float('nan'))
        ty = best.get('y', float('nan'))

        msg = Float64MultiArray()
        msg.data = [
            float(self.state),
            float(invalid_dur),
            float(self.battery_pct),
            float(len(self.variance_buffer)),
            float(best.get('x', float('nan'))),
            float(best.get('y', float('nan'))),
            float(best.get('var', float('nan'))),
            float(best_age),
            float(tx) if self.state == FS.LANDING_COMMITTED else float('nan'),
            float(ty) if self.state == FS.LANDING_COMMITTED else float('nan'),
        ]
        self.pub_diag.publish(msg)


def _variance(samples):
    n = len(samples)
    if n < 2:
        return 0.0
    m = sum(samples) / n
    return sum((s - m) ** 2 for s in samples) / (n - 1)


def main(args=None):
    rclpy.init(args=args)
    node = FailsafeNode()
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
