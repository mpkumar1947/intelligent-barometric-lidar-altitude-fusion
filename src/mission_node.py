#!/usr/bin/env python3
"""
Mission node.

GPS-denied 18-leg mission controller. Horizontal motion is commanded as
(heading, speed, duration); altitude is closed-loop on the fused altitude
from altitude_fusion_node. We never publish position setpoints based on
GPS — the drone is assumed GPS-denied. Horizontal pose is open-loop
dead-reckoning, integrated from our own commanded velocity+heading.

Leg types: CLIMB, TRANSIT, ORBIT, LAND.

Interaction with failsafe_node: this node is the only publisher to
/mavros/setpoint_raw/local. When /failsafe/state >= LIDAR_FAILED we
forward /failsafe/command into that topic instead of our own leg command.
No contention — single writer always.

Unified transit altitude: 80 m above launch. All biome transits happen at
80 m, so any altitude variation during transits must come from LiDAR
picking up different ground heights underneath. That's the paper's
headline visual.
"""

import math
import os
import pickle
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from std_msgs.msg import Float64, Float64MultiArray, Int8

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

try:
    from mpl_toolkits.mplot3d import Axes3D   # noqa: F401
    _HAS_3D = True
except Exception:
    _HAS_3D = False


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


# --- altitudes (metres AGL) ---
TAKEOFF_ALT    = 15.0
TRANSIT_ALT    = 80.0
CITY_ALT       = 70.0
VEGETATION_ALT = 30.0
TERRAIN_ALT    = 75.0
HOME_ALT       = 15.0

# CLIMB completion tuning.
CLIMB_ACCEPT_M = 1.0
CLIMB_SETTLE_S = 1.0
ALT_HOLD_TOL_M = 3.0


class LegType(IntEnum):
    CLIMB   = 0
    TRANSIT = 1
    ORBIT   = 2
    LAND    = 3


class Biome(IntEnum):
    HOME       = 0
    TRANSIT    = 1
    CITY       = 2
    VEGETATION = 3
    TERRAIN    = 4


class Leg:
    __slots__ = ('name', 'ltype', 'biome', 'target_alt',
                 'heading_deg', 'speed_mps', 'yaw_rate_dps', 'duration_s')

    def __init__(self, name, ltype, biome, target_alt,
                 heading_deg, speed_mps, yaw_rate_dps, duration_s):
        self.name = name
        self.ltype = ltype
        self.biome = biome
        self.target_alt = float(target_alt)
        self.heading_deg = float(heading_deg)
        self.speed_mps = float(speed_mps)
        self.yaw_rate_dps = float(yaw_rate_dps)
        self.duration_s = float(duration_s)   # 0 = until-condition (CLIMB)


ORBIT_SPEED_DEF   = 3.0
ORBIT_YAW_RATE    = 15.0
ORBIT_DURATION    = 48.0    # 2 laps at 15°/s
TRANSIT_SPEED_DEF = 4.0

# 18-leg schedule. Distances are dead-reckoned (our commanded velocity)
# so the biome "centres" in BIOME_META are where we think we are, not
# necessarily where Gazebo places us — the gap between those is the
# GPS-denied story.
LEGS = [
    Leg('L01', LegType.CLIMB,   Biome.TRANSIT,    TRANSIT_ALT,     0.0,   0.0,              0.0,            0.0),

    # CITY
    Leg('L02', LegType.TRANSIT, Biome.TRANSIT,    TRANSIT_ALT,   180.0,   TRANSIT_SPEED_DEF, 0.0,           25.0),
    Leg('L03', LegType.CLIMB,   Biome.CITY,       CITY_ALT,      180.0,   0.0,              0.0,            0.0),
    Leg('L04', LegType.ORBIT,   Biome.CITY,       CITY_ALT,        0.0,   ORBIT_SPEED_DEF,  ORBIT_YAW_RATE, ORBIT_DURATION),

    # Transit to VEGETATION
    Leg('L05', LegType.CLIMB,   Biome.TRANSIT,    TRANSIT_ALT,   180.0,   0.0,              0.0,            0.0),
    Leg('L06', LegType.TRANSIT, Biome.TRANSIT,    TRANSIT_ALT,     0.0,   TRANSIT_SPEED_DEF, 0.0,           25.0),
    Leg('L07', LegType.TRANSIT, Biome.TRANSIT,    TRANSIT_ALT,    90.0,   TRANSIT_SPEED_DEF, 0.0,           22.0),
    Leg('L08', LegType.CLIMB,   Biome.VEGETATION, VEGETATION_ALT, 90.0,   0.0,              0.0,            0.0),
    Leg('L09', LegType.ORBIT,   Biome.VEGETATION, VEGETATION_ALT,  0.0,   ORBIT_SPEED_DEF,  ORBIT_YAW_RATE, ORBIT_DURATION),

    # Transit to TERRAIN
    Leg('L10', LegType.CLIMB,   Biome.TRANSIT,    TRANSIT_ALT,    90.0,   0.0,              0.0,            0.0),
    Leg('L11', LegType.TRANSIT, Biome.TRANSIT,    TRANSIT_ALT,   270.0,   TRANSIT_SPEED_DEF, 0.0,           22.0),
    Leg('L12', LegType.TRANSIT, Biome.TRANSIT,    TRANSIT_ALT,     0.0,   TRANSIT_SPEED_DEF, 0.0,           38.0),
    Leg('L13', LegType.CLIMB,   Biome.TERRAIN,    TERRAIN_ALT,     0.0,   0.0,              0.0,            0.0),
    Leg('L14', LegType.ORBIT,   Biome.TERRAIN,    TERRAIN_ALT,     0.0,   ORBIT_SPEED_DEF,  ORBIT_YAW_RATE, ORBIT_DURATION),

    # Return and land
    Leg('L15', LegType.CLIMB,   Biome.TRANSIT,    TRANSIT_ALT,     0.0,   0.0,              0.0,            0.0),
    Leg('L16', LegType.TRANSIT, Biome.TRANSIT,    TRANSIT_ALT,   180.0,   TRANSIT_SPEED_DEF, 0.0,           38.0),
    Leg('L17', LegType.CLIMB,   Biome.HOME,       HOME_ALT,      180.0,   0.0,              0.0,            0.0),
    Leg('L18', LegType.LAND,    Biome.HOME,         0.0,         180.0,   0.0,              0.0,            0.0),
]


# Biome markers for plots only — not used for navigation.
BIOME_META = {
    Biome.CITY:       {'cx':  0.0, 'cy': -100.0, 'alt': CITY_ALT,       'colour': '#2196F3'},
    Biome.VEGETATION: {'cx': 90.0, 'cy':    0.0, 'alt': VEGETATION_ALT, 'colour': '#4CAF50'},
    Biome.TERRAIN:    {'cx':  0.0, 'cy':  150.0, 'alt': TERRAIN_ALT,    'colour': '#FF5722'},
}


COLORS = {
    'true':     '#ffffff',
    'lidar':    '#4fc3f7',
    'baro':     '#ffb74d',
    'ekf1':     '#81c784',
    'acf':      '#ce93d8',
    'ekf2':     '#ef5350',
    'sigma':    '#ffd54f',
    'R':        '#80deea',
    'pose':     '#f48fb1',
    'dr':       '#00e5ff',
    'gt':       '#ffffff',
    'bg':       '#1a1d27',
    'grid':     '#2a2d3a',
    'panel':    '#0f1117',
    'failsafe': '#ff1744',
}


class Phase:
    WAIT_FCU          = "WAIT_FCU"
    PRE_STREAM        = "PRE_STREAM"
    SET_GUIDED        = "SET_GUIDED"
    ARM               = "ARM"
    TAKEOFF           = "TAKEOFF"
    LEG_EXECUTE       = "LEG_EXECUTE"
    LAND_PHASE        = "LAND_PHASE"
    FAILSAFE_OVERRIDE = "FAILSAFE_OVERRIDE"
    DONE              = "DONE"


# Must match failsafe_node.FS.
class FS:
    NOMINAL           = 0
    LIDAR_DEGRADED    = 1
    LIDAR_FAILED      = 2
    BATTERY_LOW       = 3
    LANDING_COMMITTED = 4
    LANDED            = 5


# Compass convention: 0=N, 90=E, CW. MAVROS ENU yaw is measured CCW from +X.
def heading_to_enu_velocity(speed, heading_deg):
    h = math.radians(heading_deg)
    return speed * math.sin(h), speed * math.cos(h)


def heading_to_enu_yaw(heading_deg):
    return math.radians(90.0 - heading_deg)


# PositionTarget type_mask bits.
IGNORE_PX, IGNORE_PY, IGNORE_PZ     = 1, 2, 4
IGNORE_VX, IGNORE_VY, IGNORE_VZ     = 8, 16, 32
IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ  = 64, 128, 256
FORCE                               = 512
IGNORE_YAW, IGNORE_YAW_RATE         = 1024, 2048

# Velocity-XY + position-Z + yaw (for TRANSIT/ORBIT).
MASK_VEL_XY_POS_Z_YAW = (IGNORE_PX | IGNORE_PY | IGNORE_VZ
                         | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
                         | IGNORE_YAW_RATE)

# Pure position + yaw (for CLIMB). ArduCopter's GUIDED is more reliable
# with pure position targets than mixed velocity+z — the mixed form sometimes
# refuses to actuate Z when horizontal velocity is explicitly zero.
MASK_POS_XYZ_YAW = (IGNORE_VX | IGNORE_VY | IGNORE_VZ
                    | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
                    | IGNORE_YAW_RATE)


def build_setpoint(stamp, target_alt, vx_east, vy_north, heading_deg):
    # Velocity-XY + position-Z + yaw.
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


def build_setpoint_posonly(stamp, x, y, target_alt, heading_deg):
    # Pure position + yaw.
    m = PositionTarget()
    m.header.stamp = stamp
    m.header.frame_id = 'map'
    m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    m.type_mask = MASK_POS_XYZ_YAW
    m.position.x = float(x)
    m.position.y = float(y)
    m.position.z = float(target_alt)
    m.yaw = float(heading_to_enu_yaw(heading_deg))
    return m


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('transit_speed_mps',    TRANSIT_SPEED_DEF)
        self.declare_parameter('orbit_speed_mps',      ORBIT_SPEED_DEF)
        self.declare_parameter('orbit_yaw_rate_dps',   ORBIT_YAW_RATE)
        self.declare_parameter('climb_accept_alt_m',   CLIMB_ACCEPT_M)
        self.declare_parameter('alt_hold_tolerance_m', ALT_HOLD_TOL_M)

        # --- runtime state ---
        self.fcu = State()
        self.fused_alt = 0.0
        self.pose = (0.0, 0.0, 0.0)

        self.phase = Phase.WAIT_FCU
        self.phase_t0 = None

        # Leg tracking.
        self.leg_idx = 0
        self.leg_t0 = None
        self.leg_settle_t0 = None
        self.cmd_heading = 0.0    # integrates with yaw_rate during ORBIT
        self.leg_tgt_x = 0.0      # virtual target position for TRANSIT/ORBIT
        self.leg_tgt_y = 0.0
        self._leg_start_alt_delta = None

        # Dead-reckoned horizontal position (plots + failsafe only, never for control).
        self.dr_x = 0.0
        self.dr_y = 0.0
        self.dr_last_t = None

        # Failsafe.
        self.fs_state = FS.NOMINAL
        self.fs_cmd = None
        self.pre_override_leg_idx = None
        self.pre_override_leg_t_el = 0.0

        # --- logging buffers ---
        self.traj_x, self.traj_y, self.traj_z, self.traj_t = [], [], [], []

        # Gazebo ground-truth pose (/gz/iris/pose). Empty if the bridge isn't
        # running; RMSE analysis falls back to traj_* in that case.
        self.gt_x, self.gt_y, self.gt_z, self.gt_t = [], [], [], []

        self.dr_log_x, self.dr_log_y, self.dr_log_t = [], [], []

        # Fusion diagnostics — channel map must match altitude_fusion_node:
        #   [0] raw lidar slant    [1] tilt-corrected vertical AGL
        #   [2] Stage-1 baro       [3] Stage-2A ACF     [4] Stage-2B EKF-2
        #   [5] vz                 [6] adaptive R       [7] σ
        #   [8] cos_tilt           [9] az_bias
        # We only keep the ones the plots need.
        self.diag_t = []
        self.diag_lidar_raw = []
        self.diag_lidar_tilt = []
        self.diag_baro = []
        self.diag_acf = []
        self.diag_ekf2 = []
        self.diag_R = []
        self.diag_sigma = []

        self.fs_log_t = []
        self.fs_log_state = []

        # --- subscriptions ---
        self.create_subscription(
            State, '/mavros/state',
            lambda m: setattr(self, 'fcu', m), MAVROS_QOS)
        self.create_subscription(
            Float64, '/fusion/stage2b/ekf2_altitude',
            lambda m: setattr(self, 'fused_alt', m.data), MAVROS_QOS)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._cb_pose, MAVROS_QOS)
        # Ground truth — RELIABLE matches the bridge's advertisement.
        self.create_subscription(
            PoseStamped, '/gz/iris/pose',
            self._cb_gt_pose, RELIABLE_QOS)
        self.create_subscription(
            Float64MultiArray, '/fusion/diagnostics',
            self._cb_diag, MAVROS_QOS)
        self.create_subscription(
            Int8, '/failsafe/state',
            self._cb_failsafe_state, RELIABLE_QOS)
        self.create_subscription(
            PositionTarget, '/failsafe/command',
            self._cb_failsafe_cmd, RELIABLE_QOS)

        # --- publishers ---
        # /mavros/setpoint_raw/local has a single writer across the whole stack
        # (this node); failsafe goes through us.
        self.pub_sp  = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.pub_leg = self.create_publisher(Float64MultiArray, '/mission/leg_info', RELIABLE_QOS)
        self.pub_dr  = self.create_publisher(PoseStamped, '/mission/dead_reckoned_pose', 10)

        self.cli_arm     = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_mode    = self.create_client(SetMode,     '/mavros/set_mode')
        self.cli_takeoff = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')

        # 20 Hz setpoint, 2 Hz FSM + leg info, 10 Hz DR pose.
        self.create_timer(0.05, self._cb_sp)
        self.create_timer(0.5,  self._cb_fsm)
        self.create_timer(0.5,  self._cb_leg_info)
        self.create_timer(0.1,  self._cb_dr_pub)

        self._wait_for_services()

        self.get_logger().info(
            f"[mission] up. {len(LEGS)} legs, transit_alt={TRANSIT_ALT:.0f}m")
        for lg in LEGS:
            dur = f"{lg.duration_s:>5.1f}s" if lg.duration_s > 0 else " cond "
            self.get_logger().info(
                f"  {lg.name} {lg.ltype.name:<7s} {lg.biome.name:<10s} "
                f"alt={lg.target_alt:>5.1f} hdg={lg.heading_deg:>5.1f} "
                f"v={lg.speed_mps:>3.1f} yr={lg.yaw_rate_dps:>5.1f} {dur}")

    # --- services / clock ------------------------------------------------

    def _wait_for_services(self):
        self.get_logger().info('Waiting for MAVROS services...')
        for client, name in [
            (self.cli_arm, '/mavros/cmd/arming'),
            (self.cli_mode, '/mavros/set_mode'),
            (self.cli_takeoff, '/mavros/cmd/takeoff'),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'  ...still waiting for {name}')
        self.get_logger().info('MAVROS services ready')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self):
        return self._now() - (self.phase_t0 or self._now())

    def _leg_elapsed(self):
        return self._now() - (self.leg_t0 or self._now())

    def _enter(self, phase):
        self.phase = phase
        self.phase_t0 = self._now()
        self.get_logger().info(f'-> {phase}')

    def _enter_leg(self, idx):
        self.leg_idx = idx
        self.leg_t0 = self._now()
        self.leg_settle_t0 = None
        self._leg_start_alt_delta = None
        if idx < len(LEGS):
            lg = LEGS[idx]
            self.cmd_heading = lg.heading_deg
            # Seed the virtual target at our current actual pose so we chase
            # from where we actually are, not from a stale position.
            self.leg_tgt_x = self.pose[0]
            self.leg_tgt_y = self.pose[1]
            self.get_logger().info(
                f'  leg {lg.name} {lg.ltype.name}/{lg.biome.name} '
                f'alt={lg.target_alt:.1f} hdg={lg.heading_deg:.1f} '
                f'dur={lg.duration_s:.1f} seed=({self.leg_tgt_x:.1f},{self.leg_tgt_y:.1f})')

    # --- subscriber callbacks --------------------------------------------

    def _cb_pose(self, msg):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)
        if self.phase in (Phase.TAKEOFF, Phase.LEG_EXECUTE,
                          Phase.LAND_PHASE, Phase.FAILSAFE_OVERRIDE, Phase.DONE):
            self.traj_x.append(p.x)
            self.traj_y.append(p.y)
            self.traj_z.append(p.z)
            self.traj_t.append(self._now())

    def _cb_gt_pose(self, msg):
        # Gated to the same phases as _cb_pose so gt_t/traj_t span identical
        # intervals and RMSE comparisons are apples-to-apples.
        p = msg.pose.position
        if self.phase in (Phase.TAKEOFF, Phase.LEG_EXECUTE,
                          Phase.LAND_PHASE, Phase.FAILSAFE_OVERRIDE, Phase.DONE):
            self.gt_x.append(p.x)
            self.gt_y.append(p.y)
            self.gt_z.append(p.z)
            self.gt_t.append(self._now())

    def _cb_diag(self, msg):
        if self.phase not in (Phase.TAKEOFF, Phase.LEG_EXECUTE,
                              Phase.LAND_PHASE, Phase.FAILSAFE_OVERRIDE, Phase.DONE):
            return
        d = msg.data
        if len(d) < 8:
            return
        self.diag_t.append(self._now())
        self.diag_lidar_raw.append(d[0])
        self.diag_lidar_tilt.append(d[1])
        self.diag_baro.append(d[2])
        self.diag_acf.append(d[3])
        self.diag_ekf2.append(d[4])
        self.diag_R.append(d[6])
        self.diag_sigma.append(d[7])

    def _cb_failsafe_state(self, msg):
        new_state = int(msg.data)
        if new_state != self.fs_state:
            self.get_logger().warn(f'[failsafe] state {self.fs_state} -> {new_state}')
        self.fs_state = new_state
        self.fs_log_t.append(self._now())
        self.fs_log_state.append(new_state)

    def _cb_failsafe_cmd(self, msg):
        self.fs_cmd = msg

    # --- 20 Hz setpoint stream -------------------------------------------
    # ArduCopter's GUIDED reliably chases position targets but unreliably
    # honours velocity commands via /setpoint_raw/local. So for TRANSIT/ORBIT
    # we maintain a "virtual target" position that advances at the commanded
    # speed every tick, and publish its position — same ground track, but
    # through the position controller.

    def _cb_sp(self):
        stamp = self.get_clock().now().to_msg()

        # Pre-takeoff: stay silent. Streaming setpoints before ARM confuses
        # ArduCopter's arming preflight checks.
        if self.phase in (Phase.WAIT_FCU, Phase.PRE_STREAM, Phase.SET_GUIDED,
                          Phase.ARM, Phase.TAKEOFF):
            return

        if self.phase == Phase.FAILSAFE_OVERRIDE:
            if self.fs_cmd is not None:
                fwd = self.fs_cmd
                fwd.header.stamp = stamp
                self.pub_sp.publish(fwd)
            else:
                # Race: we entered override but failsafe hasn't published its
                # first /failsafe/command yet. Hold current pose.
                px, py, _ = self.pose
                self.pub_sp.publish(build_setpoint_posonly(
                    stamp, px, py, self.fused_alt, self.cmd_heading))
            return

        if self.phase == Phase.LEG_EXECUTE and self.leg_idx < len(LEGS):
            lg = LEGS[self.leg_idx]

            if lg.ltype == LegType.CLIMB:
                # Hold current horizontal, move to target altitude.
                px, py, _ = self.pose
                self.pub_sp.publish(build_setpoint_posonly(
                    stamp, px, py, lg.target_alt, self.cmd_heading))
                self.get_logger().info(
                    f'  [sp CLIMB] ({px:.1f},{py:.1f},{lg.target_alt:.1f}) '
                    f'yaw={self.cmd_heading:.0f} cur_z={self.pose[2]:.1f}',
                    throttle_duration_sec=2.0)
                return

            # TRANSIT / ORBIT: advance virtual target, publish its position.
            dt_tick = 0.05

            if lg.ltype == LegType.ORBIT:
                self.cmd_heading = (self.cmd_heading
                                    + lg.yaw_rate_dps * dt_tick) % 360.0

            vx, vy = heading_to_enu_velocity(lg.speed_mps, self.cmd_heading)
            self.leg_tgt_x += vx * dt_tick
            self.leg_tgt_y += vy * dt_tick

            # Leash the virtual target so it can't run off ahead of the drone.
            # If ArduCopter's WPNAV_SPEED is below our commanded speed, the
            # drone falls behind; without a leash the target would escape
            # into infinity after any stall.
            MAX_LEAD_M = 8.0
            dxt = self.leg_tgt_x - self.pose[0]
            dyt = self.leg_tgt_y - self.pose[1]
            lead = math.hypot(dxt, dyt)
            if lead > MAX_LEAD_M:
                scale = MAX_LEAD_M / lead
                self.leg_tgt_x = self.pose[0] + dxt * scale
                self.leg_tgt_y = self.pose[1] + dyt * scale

            self.pub_sp.publish(build_setpoint_posonly(
                stamp, self.leg_tgt_x, self.leg_tgt_y,
                lg.target_alt, self.cmd_heading))

            self.get_logger().info(
                f'  [sp {lg.ltype.name}] tgt=({self.leg_tgt_x:.1f},'
                f'{self.leg_tgt_y:.1f},{lg.target_alt:.1f}) '
                f'hdg={self.cmd_heading:.0f} cur=({self.pose[0]:.1f},'
                f'{self.pose[1]:.1f},{self.pose[2]:.1f}) lead={lead:.1f}',
                throttle_duration_sec=2.0)

            self._advance_dead_reckoning(lg.speed_mps, self.cmd_heading)
            return

        if self.phase == Phase.LAND_PHASE:
            # ArduCopter LAND owns the drone now — don't compete.
            return

    # --- dead-reckoning integrator ---------------------------------------
    # Intent-integral: feed what we asked the drone to do, not what the IMU
    # sees. The gap between this and MAVROS pose IS the story.

    def _advance_dead_reckoning(self, speed, heading_deg):
        now = self._now()
        if self.dr_last_t is None:
            self.dr_last_t = now
            return
        dt = now - self.dr_last_t
        self.dr_last_t = now
        if speed > 0.0 and dt < 0.5:
            vx, vy = heading_to_enu_velocity(speed, heading_deg)
            self.dr_x += vx * dt
            self.dr_y += vy * dt
            self.dr_log_x.append(self.dr_x)
            self.dr_log_y.append(self.dr_y)
            self.dr_log_t.append(now)

    def _cb_dr_pub(self):
        if self.phase not in (Phase.LEG_EXECUTE, Phase.FAILSAFE_OVERRIDE,
                              Phase.LAND_PHASE, Phase.DONE):
            return
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x = self.dr_x
        m.pose.position.y = self.dr_y
        m.pose.position.z = self.fused_alt
        m.pose.orientation.w = 1.0
        self.pub_dr.publish(m)

    def _cb_leg_info(self):
        if self.phase != Phase.LEG_EXECUTE or self.leg_idx >= len(LEGS):
            return
        lg = LEGS[self.leg_idx]
        msg = Float64MultiArray()
        msg.data = [
            float(self.leg_idx),
            float(lg.ltype),
            float(lg.biome),
            float(lg.target_alt),
            float(self._leg_elapsed()),
            float(lg.duration_s) if lg.duration_s > 0.0 else -1.0,
            float(self.cmd_heading),
        ]
        self.pub_leg.publish(msg)

    # --- 2 Hz FSM --------------------------------------------------------

    def _cb_fsm(self):
        # Failsafe override is a top-level interrupt.
        if self.phase in (Phase.LEG_EXECUTE, Phase.LAND_PHASE):
            if self.fs_state >= FS.LIDAR_FAILED:
                self.pre_override_leg_idx = self.leg_idx
                self.pre_override_leg_t_el = self._leg_elapsed()
                self.get_logger().warn(
                    f'[failsafe] OVERRIDE from {self.phase} '
                    f'(leg {LEGS[self.leg_idx].name if self.leg_idx < len(LEGS) else "-"})')
                self._enter(Phase.FAILSAFE_OVERRIDE)
                return

        if self.phase == Phase.FAILSAFE_OVERRIDE:
            if self.fs_state == FS.NOMINAL and self.pre_override_leg_idx is not None:
                idx = self.pre_override_leg_idx
                self.get_logger().info(
                    f'[failsafe] cleared -> resuming at {LEGS[idx].name}')
                self.pre_override_leg_idx = None
                self.pre_override_leg_t_el = 0.0
                self._enter_leg(idx)
                self._enter(Phase.LEG_EXECUTE)
                return
            if self.fs_state == FS.LANDED:
                self.get_logger().info('[failsafe] LANDED -> mission done')
                self._enter(Phase.DONE)
                self._generate_plots()
                return
            return

        if self.phase == Phase.WAIT_FCU:
            if self.fcu.connected:
                self.get_logger().info('FCU connected')
                self._enter(Phase.PRE_STREAM)

        elif self.phase == Phase.PRE_STREAM:
            # Brief settle before mode change.
            if self._elapsed() >= 1.0:
                self._enter(Phase.SET_GUIDED)

        elif self.phase == Phase.SET_GUIDED:
            if self.fcu.mode == 'GUIDED':
                self._enter(Phase.ARM)
            elif self._elapsed() < 0.6 or self._elapsed() > 4.0:
                self._set_mode('GUIDED')
                if self._elapsed() > 4.0:
                    self.phase_t0 = self._now()

        elif self.phase == Phase.ARM:
            if self.fcu.armed:
                self.get_logger().info('armed')
                self._send_takeoff()
                # Arm = origin for GPS-denied nav.
                self.dr_x = 0.0
                self.dr_y = 0.0
                self.dr_last_t = self._now()
                self._enter(Phase.TAKEOFF)
            elif self._elapsed() < 0.6 or self._elapsed() > 5.0:
                self._arm(True)
                if self._elapsed() > 5.0:
                    self.phase_t0 = self._now()

        elif self.phase == Phase.TAKEOFF:
            z = self.pose[2]
            self.get_logger().info(
                f'  climb {z:.1f}/{TAKEOFF_ALT}m (mode={self.fcu.mode} armed={self.fcu.armed} el={self._elapsed():.0f}s)',
                throttle_duration_sec=2.0)

            # CommandTOL often stops short of the commanded altitude; 85% is fine.
            if z >= TAKEOFF_ALT * 0.85:
                self.get_logger().info(f'  at {z:.1f}m -> starting leg sequence')
                self._enter_leg(0)
                self._enter(Phase.LEG_EXECUTE)

            elif not self.fcu.armed:
                self.get_logger().warn('disarmed during climb — re-arming')
                self._arm(True)
                self.phase_t0 = self._now()

            elif self._elapsed() > 10.0 and z < 2.0:
                # CommandTOL stalled (common in SITL when EKF isn't settled).
                # Just drop into LEG_EXECUTE — L01 is a CLIMB that will drive us
                # the rest of the way up via streamed position setpoints.
                self.get_logger().warn(
                    f'CommandTOL stalled at z={z:.1f}m — falling back to setpoint-stream climb')
                self._enter_leg(0)
                self._enter(Phase.LEG_EXECUTE)

            elif self._elapsed() > 30.0:
                self.get_logger().warn(
                    f'takeoff stalled at z={z:.1f}/{TAKEOFF_ALT} after 30s — '
                    f'accepting partial, L01 will finish the climb')
                self._enter_leg(0)
                self._enter(Phase.LEG_EXECUTE)

        elif self.phase == Phase.LEG_EXECUTE:
            if self.leg_idx >= len(LEGS):
                self._enter(Phase.DONE)
                self.get_logger().info('MISSION COMPLETE')
                self._generate_plots()
                return

            lg = LEGS[self.leg_idx]
            completed = self._leg_is_complete(lg)

            # Altitude-drift warning. Compare against pose_z (EKF3, absolute
            # frame), NOT fused_alt — fused_alt tracks AGL and legitimately
            # differs from target by tens of metres over TERRAIN hills.
            # Showing both helps diagnose controller vs fusion issues.
            if lg.ltype in (LegType.TRANSIT, LegType.ORBIT):
                pose_err  = abs(self.pose[2] - lg.target_alt)
                fused_err = abs(self.fused_alt - lg.target_alt)
                tol = self.get_parameter('alt_hold_tolerance_m').value
                if pose_err > tol:
                    self.get_logger().warn(
                        f'  [{lg.name}] pose-alt err {pose_err:.1f}m > {tol:.1f}m '
                        f'(fused-err={fused_err:.1f}m, continuing)',
                        throttle_duration_sec=3.0)

            if completed:
                self.get_logger().info(
                    f'  leg {lg.name} done  pose=({self.pose[0]:.1f},{self.pose[1]:.1f},{self.pose[2]:.2f}) '
                    f'fused={self.fused_alt:.2f}  dr=({self.dr_x:.1f},{self.dr_y:.1f})')
                nxt = self.leg_idx + 1
                if nxt < len(LEGS) and LEGS[nxt].ltype == LegType.LAND:
                    self._enter_leg(nxt)
                    self._enter(Phase.LAND_PHASE)
                    self._set_mode('LAND')
                else:
                    self._enter_leg(nxt)

        elif self.phase == Phase.LAND_PHASE:
            if not self.fcu.armed:
                self.get_logger().info('disarmed — ground contact')
                self._enter(Phase.DONE)
                self._generate_plots()

    # --- leg completion --------------------------------------------------

    def _leg_is_complete(self, lg):
        if lg.ltype == LegType.CLIMB:
            # CLIMB completion uses MAVROS pose Z, NOT fused_alt. Fused_alt is
            # LiDAR-dominant AGL; over TERRAIN biome with 15m hills, fused_alt
            # reads ~65m while the drone is actually at 80m absolute — correct
            # AGL behaviour but useless as "have I reached the commanded Z?"
            # Pose Z comes from EKF3 in the same absolute frame as the target.
            accept = self.get_parameter('climb_accept_alt_m').value
            pose_z = self.pose[2]
            if abs(pose_z - lg.target_alt) < accept:
                if self.leg_settle_t0 is None:
                    self.leg_settle_t0 = self._now()
                elif (self._now() - self.leg_settle_t0) >= CLIMB_SETTLE_S:
                    return True
            else:
                self.leg_settle_t0 = None

            # Adaptive timeout. ArduCopter defaults: up ~2.5 m/s, down ~1.5 m/s.
            # A 50m descent takes ~33s just for transit, so a fixed 20s
            # timeout fired prematurely on the big transitions (80->30 etc).
            # Budget = 20s base + 1s per metre of altitude delta, cap 90s.
            if self._leg_start_alt_delta is None:
                self._leg_start_alt_delta = abs(pose_z - lg.target_alt)
            timeout_s = min(20.0 + self._leg_start_alt_delta * 1.0, 90.0)
            if self._leg_elapsed() > timeout_s:
                self.get_logger().warn(
                    f'  [{lg.name}] CLIMB timeout @ {self._leg_elapsed():.1f}s '
                    f'(budget {timeout_s:.0f}s for {self._leg_start_alt_delta:.0f}m) '
                    f'pose_z={pose_z:.1f} target={lg.target_alt:.1f} — accepting')
                self._leg_start_alt_delta = None
                return True
            return False

        if lg.ltype in (LegType.TRANSIT, LegType.ORBIT):
            return self._leg_elapsed() >= lg.duration_s

        if lg.ltype == LegType.LAND:
            return not self.fcu.armed

        return False

    # --- MAVROS helpers --------------------------------------------------

    def _set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.cli_mode.call_async(req)

    def _arm(self, val):
        req = CommandBool.Request()
        req.value = val
        self.cli_arm.call_async(req)

    def _send_takeoff(self):
        req = CommandTOL.Request()
        req.altitude = TAKEOFF_ALT
        self.cli_takeoff.call_async(req)
        self.get_logger().info(f'MAV_CMD_NAV_TAKEOFF -> {TAKEOFF_ALT}m')

    # --- plot generation -------------------------------------------------

    def _generate_plots(self):
        self.get_logger().info('generating plots...')
        if not self.traj_x:
            self.get_logger().warn('no trajectory data — skipping plots')
            return

        traj_x = np.array(self.traj_x)
        traj_y = np.array(self.traj_y)
        traj_z = np.array(self.traj_z)
        traj_t = np.array(self.traj_t)
        t0 = traj_t[0] if len(traj_t) else 0.0
        traj_t_rel = traj_t - t0

        has_diag = len(self.diag_t) > 0
        if has_diag:
            diag_t          = np.array(self.diag_t) - t0
            diag_lidar_raw  = np.array(self.diag_lidar_raw)
            diag_lidar_tilt = np.array(self.diag_lidar_tilt)
            diag_baro       = np.array(self.diag_baro)
            diag_acf        = np.array(self.diag_acf)
            diag_ekf2       = np.array(self.diag_ekf2)
            diag_R          = np.array(self.diag_R)
            diag_sigma      = np.array(self.diag_sigma)

        dr_x = np.array(self.dr_log_x) if self.dr_log_x else np.array([])
        dr_y = np.array(self.dr_log_y) if self.dr_log_y else np.array([])

        fs_t = np.array(self.fs_log_t) - t0 if self.fs_log_t else np.array([])
        fs_s = np.array(self.fs_log_state) if self.fs_log_state else np.array([])

        def styled_ax(ax, title, xlabel='', ylabel=''):
            ax.set_facecolor(COLORS['bg'])
            ax.set_title(title, color='white', fontsize=8, pad=5)
            ax.tick_params(colors='#888888', labelsize=7)
            for sp in ax.spines.values():
                sp.set_edgecolor(COLORS['grid'])
            ax.grid(color=COLORS['grid'], linewidth=0.5)
            if xlabel:
                ax.set_xlabel(xlabel, color='white', fontsize=7)
            if ylabel:
                ax.set_ylabel(ylabel, color='white', fontsize=7)
            ax.legend(fontsize=7, framealpha=0.3,
                      facecolor=COLORS['panel'], labelcolor='white')

        def shade_failsafe(ax):
            if len(fs_t) < 2:
                return
            i = 0
            while i < len(fs_t):
                if fs_s[i] != FS.NOMINAL:
                    start = fs_t[i]
                    while i < len(fs_t) and fs_s[i] != FS.NOMINAL:
                        i += 1
                    end = fs_t[i - 1] if i > 0 else fs_t[-1]
                    ax.axvspan(start, end, color=COLORS['failsafe'],
                               alpha=0.12, zorder=0)
                else:
                    i += 1

        fig = plt.figure(figsize=(16, 20))
        fig.patch.set_facecolor(COLORS['panel'])
        gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.42, wspace=0.28)

        # Panel 1 — bird's-eye with dead-reckoned overlay.
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.set_facecolor(COLORS['bg'])
        ax1.set_aspect('equal')
        ax1.set_title("Bird's-Eye  ·  MAVROS pose vs dead-reckoned",
                      color='white', fontsize=8, pad=5)
        ax1.tick_params(colors='#888888', labelsize=7)
        for sp in ax1.spines.values():
            sp.set_edgecolor(COLORS['grid'])
        ax1.grid(color=COLORS['grid'], linewidth=0.5)
        ax1.plot(traj_x, traj_y, color=COLORS['pose'], linewidth=1.2,
                 alpha=0.9, label='MAVROS pose (EKF3)')
        if len(dr_x):
            ax1.plot(dr_x, dr_y, color=COLORS['dr'], linewidth=1.2,
                     linestyle='--', alpha=0.9, label='Dead-reckoned')
        ax1.scatter(traj_x[0], traj_y[0], c='#00E676', s=100, zorder=6,
                    marker='o', edgecolors='white', linewidths=1.2, label='Takeoff')
        ax1.scatter(traj_x[-1], traj_y[-1], c='#FF1744', s=100, zorder=6,
                    marker='X', edgecolors='white', linewidths=1.2, label='Landing')
        for b, meta in BIOME_META.items():
            ax1.scatter(meta['cx'], meta['cy'], s=300, marker='*',
                        c=meta['colour'], edgecolors='white', linewidths=1.2,
                        alpha=0.7, zorder=5)
            ax1.text(meta['cx'] + 5, meta['cy'] + 5,
                     f"{b.name}\n{meta['alt']:.0f}m",
                     fontsize=7, fontweight='bold', color=meta['colour'])
        ax1.set_xlabel('East X (m)', color='white', fontsize=7)
        ax1.set_ylabel('North Y (m)', color='white', fontsize=7)
        # Legend below axis — mission spans y=[-100, 160] so any in-axis
        # placement overlaps the path.
        ax1.legend(fontsize=7, framealpha=0.35,
                   facecolor=COLORS['panel'], labelcolor='white',
                   loc='upper center', bbox_to_anchor=(0.5, -0.12),
                   ncol=2, borderaxespad=0.2)

        # Panel 2 — 3-D.
        if _HAS_3D:
            ax2 = fig.add_subplot(gs[0, 1], projection='3d')
            ax2.set_facecolor(COLORS['bg'])
            ax2.plot(traj_x, traj_y, traj_z, color='#ce93d8',
                     linewidth=1.4, label='3-D path')
            ax2.scatter(traj_x[0], traj_y[0], traj_z[0], c='#00E676', s=60, zorder=5)
            ax2.scatter(traj_x[-1], traj_y[-1], traj_z[-1],
                        c='#FF1744', s=60, zorder=5, marker='X')
            ax2.set_xlabel('X (m)', color='white', fontsize=7, labelpad=4)
            ax2.set_ylabel('Y (m)', color='white', fontsize=7, labelpad=4)
            ax2.set_zlabel('Alt (m)', color='white', fontsize=7, labelpad=4)
            ax2.set_title('3-D Trajectory', color='white', fontsize=8, pad=5)
            ax2.tick_params(colors='#888888', labelsize=7)
            ax2.xaxis.pane.fill = False
            ax2.yaxis.pane.fill = False
            ax2.zaxis.pane.fill = False
            ax2.xaxis.pane.set_edgecolor(COLORS['grid'])
            ax2.yaxis.pane.set_edgecolor(COLORS['grid'])
            ax2.zaxis.pane.set_edgecolor(COLORS['grid'])
            ax2.grid(color=COLORS['grid'], linewidth=0.5)
            ax2.legend(fontsize=7, framealpha=0.3,
                       facecolor=COLORS['panel'], labelcolor='white')

        # Panel 3 — raw sensors vs pose.
        ax3 = fig.add_subplot(gs[1, 0])
        shade_failsafe(ax3)
        ax3.plot(traj_t_rel, traj_z, color=COLORS['pose'], lw=1.4,
                 label='MAVROS pose Z (ref)')
        if has_diag:
            ax3.scatter(diag_t, diag_lidar_raw, s=3, color=COLORS['lidar'],
                        alpha=0.35, label='LiDAR raw (slant)')
            ax3.scatter(diag_t, diag_lidar_tilt, s=3, color='#81d4fa',
                        alpha=0.7, label='LiDAR tilt-corr (vertical AGL)')
            ax3.plot(diag_t, diag_baro, color=COLORS['baro'], lw=1.2,
                     alpha=0.9, label='Baro Stage-1 (fused IMU+Baro)')
        styled_ax(ax3, 'Raw Sensors vs MAVROS Pose',
                  xlabel='Time (s)', ylabel='Altitude AGL (m)')

        # Panel 4 — Stage-1 EKF-1.
        ax4 = fig.add_subplot(gs[1, 1])
        shade_failsafe(ax4)
        ax4.plot(traj_t_rel, traj_z, color=COLORS['pose'], lw=1.2, alpha=0.7,
                 label='MAVROS pose Z')
        if has_diag:
            ax4.plot(diag_t, diag_baro, color=COLORS['ekf1'], lw=1.6,
                     label='EKF-1 Stage-1 (IMU+Baro fused)')
        styled_ax(ax4, 'Stage 1 — EKF-1 (IMU + Barometer)',
                  xlabel='Time (s)', ylabel='Altitude AGL (m)')

        # Panel 5 — Stage-2A ACF.
        ax5 = fig.add_subplot(gs[2, 0])
        shade_failsafe(ax5)
        ax5.plot(traj_t_rel, traj_z, color=COLORS['pose'], lw=1.2, alpha=0.7,
                 label='MAVROS pose Z')
        if has_diag:
            ax5.plot(diag_t, diag_baro, color=COLORS['ekf1'], lw=1.0,
                     alpha=0.4, label='Stage-1 (baro)')
            ax5.plot(diag_t, diag_acf, color=COLORS['acf'], lw=1.8,
                     label='ACF Stage-2A')
        styled_ax(ax5, 'Stage 2A — Adaptive Complementary Filter',
                  xlabel='Time (s)', ylabel='Altitude AGL (m)')

        # Panel 6 — Stage-2B EKF-2 IAKF.
        # Raw trace + smoothed overlay. EKF-2 takes two measurement streams:
        # baro at ~4 Hz (ASL) and LiDAR at ~100 Hz (AGL). Over terrain where
        # ground is elevated, baro says 80m, LiDAR says 50m, filter alternates
        # ~4 times/s producing a zig-zag. Thin raw trace so oscillations are
        # visible instead of blurring into a fat band; smoothed overlay shows
        # the trend. σ is in Panel 7.
        ax6 = fig.add_subplot(gs[2, 1])
        shade_failsafe(ax6)
        ax6.plot(traj_t_rel, traj_z, color=COLORS['pose'], lw=1.2, alpha=0.7,
                 label='MAVROS pose Z (ref)')
        if has_diag:
            ax6.plot(diag_t, diag_ekf2, color=COLORS['ekf2'], lw=0.7,
                     alpha=0.5, label='EKF-2 raw (per-update)')
            if len(diag_ekf2) >= 50:
                kernel = np.ones(50) / 50.0
                diag_ekf2_smooth = np.convolve(diag_ekf2, kernel, mode='same')
                ax6.plot(diag_t, diag_ekf2_smooth, color=COLORS['ekf2'],
                         lw=1.8, label='EKF-2 smoothed (50-sample avg)')
        styled_ax(ax6, 'Stage 2B — EKF-2 with Adaptive Covariance (IAKF)',
                  xlabel='Time (s)', ylabel='Altitude AGL (m)')

        # Panel 7 — EKF-2 σ.
        ax7 = fig.add_subplot(gs[3, 0])
        if has_diag:
            ax7.plot(diag_t, diag_sigma, color=COLORS['sigma'], lw=1.4,
                     label='EKF-2 σ')
        styled_ax(ax7, 'EKF-2 — Altitude Uncertainty (1σ)',
                  xlabel='Time (s)', ylabel='σ (m)')

        # Panel 8 — adaptive R_lidar.
        ax8 = fig.add_subplot(gs[3, 1])
        if has_diag:
            ax8.plot(diag_t, diag_R, color=COLORS['R'], lw=1.2,
                     label='Adaptive R_lidar')
            ax8.axhline(0.01, color='#888888', lw=0.8, ls='--',
                        label='Spec min (0.01 m²)')
        styled_ax(ax8, 'Adaptive LiDAR Measurement Noise R',
                  xlabel='Time (s)', ylabel='R (m²)')

        fig.suptitle(
            'GPS-Denied UAV Mission  ·  TF03-180 LiDAR + IMU + Barometer\n'
            'Stage 1: EKF(IMU+Baro)   Stage 2A: Adaptive CF   Stage 2B: EKF-IAKF\n'
            'Panel 1: MAVROS vs dead-reckoned   Red shading: failsafe active',
            color='white', fontsize=11, y=0.995)

        out_main = 'mission_fusion.png'
        fig.savefig(out_main, dpi=160, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        plt.close(fig)
        self.get_logger().info(f'plot -> {out_main}')

        # Standalone high-res 3-D.
        if _HAS_3D:
            fig3d = plt.figure(figsize=(9, 7))
            fig3d.patch.set_facecolor(COLORS['panel'])
            ax3d = fig3d.add_subplot(111, projection='3d')
            ax3d.set_facecolor(COLORS['bg'])
            ax3d.plot(traj_x, traj_y, traj_z, color='#ce93d8',
                      linewidth=1.5, label='3-D MAVROS path')
            ax3d.scatter(traj_x[0], traj_y[0], traj_z[0], c='#00E676', s=80, zorder=5)
            ax3d.scatter(traj_x[-1], traj_y[-1], traj_z[-1],
                         c='#FF1744', s=80, zorder=5, marker='X')
            for b, meta in BIOME_META.items():
                ax3d.scatter(meta['cx'], meta['cy'], meta['alt'],
                             c=meta['colour'], s=120, zorder=6, marker='*')
                ax3d.text(meta['cx'], meta['cy'], meta['alt'] + 2.0,
                          b.name, color=meta['colour'], fontsize=7)
            ax3d.set_xlabel('East X (m)', color='white', fontsize=8)
            ax3d.set_ylabel('North Y (m)', color='white', fontsize=8)
            ax3d.set_zlabel('Altitude (m)', color='white', fontsize=8)
            ax3d.set_title('3-D Mission Trajectory', color='white', fontsize=10)
            ax3d.tick_params(colors='#888888', labelsize=7)
            ax3d.xaxis.pane.fill = False
            ax3d.yaxis.pane.fill = False
            ax3d.zaxis.pane.fill = False
            ax3d.xaxis.pane.set_edgecolor(COLORS['grid'])
            ax3d.yaxis.pane.set_edgecolor(COLORS['grid'])
            ax3d.zaxis.pane.set_edgecolor(COLORS['grid'])
            ax3d.legend(fontsize=8, framealpha=0.3,
                        facecolor=COLORS['panel'], labelcolor='white')
            plt.tight_layout()
            out_3d = 'mission_3d.png'
            fig3d.savefig(out_3d, dpi=160, bbox_inches='tight',
                          facecolor=fig3d.get_facecolor())
            plt.close(fig3d)
            self.get_logger().info(f'3d plot -> {out_3d}')

        # Pickle dump for offline RMSE analysis.
        # Persists all time-series buffers in one file so compute_rmse_table.py
        # can time-align fusion outputs against Gazebo ground truth. If the
        # bridge wasn't running, gt_* arrays are empty and RMSE falls back to
        # MAVROS pose (noted in the log).
        try:
            os.makedirs('logs', exist_ok=True)
            scenario = os.environ.get('SCENARIO', 'manual')
            ts = os.environ.get('TS', str(int(traj_t[0] if len(traj_t) else 0.0)))
            pkl_path = f'logs/mission_data_{scenario}_{ts}.pkl'
            payload = {
                'scenario': scenario,
                'ts': ts,
                # MAVROS pose (EKF3) — cross-check reference.
                'traj_t': list(self.traj_t),
                'traj_x': list(self.traj_x),
                'traj_y': list(self.traj_y),
                'traj_z': list(self.traj_z),
                # Gazebo ground truth — empty if bridge wasn't running.
                'gt_t': list(self.gt_t),
                'gt_x': list(self.gt_x),
                'gt_y': list(self.gt_y),
                'gt_z': list(self.gt_z),
                # Fusion diagnostics (same channels as Panels 3-8).
                'diag_t': list(self.diag_t),
                'diag_lidar_raw': list(self.diag_lidar_raw),
                'diag_lidar_tilt': list(self.diag_lidar_tilt),
                'diag_baro': list(self.diag_baro),
                'diag_acf': list(self.diag_acf),
                'diag_ekf2': list(self.diag_ekf2),
                'diag_R': list(self.diag_R),
                'diag_sigma': list(self.diag_sigma),
                'dr_t': list(self.dr_log_t),
                'dr_x': list(self.dr_log_x),
                'dr_y': list(self.dr_log_y),
                'fs_t': list(self.fs_log_t),
                'fs_s': list(self.fs_log_state),
            }
            with open(pkl_path, 'wb') as fp:
                pickle.dump(payload, fp)
            self.get_logger().info(
                f'pickle -> {pkl_path} '
                f'(traj={len(self.traj_t)} gt={len(self.gt_t)} diag={len(self.diag_t)})')
            if len(self.gt_t) == 0:
                self.get_logger().warn(
                    '  gt buffer empty — gz_pose_bridge was not publishing. '
                    'RMSE will fall back to MAVROS pose.')
        except Exception as e:
            self.get_logger().warn(f'pickle save failed: {e}')

        self.get_logger().info('plots done')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
