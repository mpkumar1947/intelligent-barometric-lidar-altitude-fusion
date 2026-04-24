#!/usr/bin/env python3
"""
Altitude sensor fusion.

Three-stage pipeline. Stage 1 fuses IMU+baro with a small EKF (EKF-1) to
produce a smoothed baro-based altitude at ~4 Hz. Stage 2 then re-fuses
that with LiDAR (Benewake TF03-180) two different ways: Stage 2A is an
Adaptive Complementary Filter, Stage 2B is a second EKF with innovation-
adaptive measurement noise (IAKF). We run them in parallel so we can
compare/ablate and so the downstream consumer can pick the best signal
for its purpose.

Sensor notes:
  TF03-180 LiDAR: 0.1-180 m, ±10 cm within 10 m / 1% beyond, ~100 Hz,
    single-beam so we only read ranges[0] of the LaserScan.
  Baro: static pressure, ~4 Hz, converted via ISA hypsometric formula.
  IMU: ~100 Hz, supplies orientation (for LiDAR tilt correction) and
    vertical acceleration (for EKF predict steps).

Ground pressure auto-calibration:
  The hardcoded ground_pressure_pa is a session-specific fallback. On the
  first disarmed->armed edge we overwrite it with the current raw baro
  reading so a fresh SITL run doesn't carry a constant altitude bias.
  Rejected with a warning if the delta looks like we're already airborne
  (fusion node restart case).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Imu, FluidPressure, LaserScan
from std_msgs.msg import Float64, Float64MultiArray
from mavros_msgs.msg import State


# The managed bridge republishes as BEST_EFFORT (inherited from MAVROS).
# If we subscribe RELIABLE we get silently zero messages. Match it.
MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ISA (International Standard Atmosphere) constants.
P0 = 101325.0    # sea-level pressure [Pa]
T0 = 288.15      # sea-level temperature [K]
L  = 0.0065      # lapse rate [K/m]
R  = 8.314462    # gas constant [J/(mol·K)]
M  = 0.028964    # molar mass of dry air [kg/mol]
G  = 9.80665     # gravity [m/s²]
ALPHA = G * M / (R * L)   # barometric exponent ≈ 5.2558


def pressure_to_altitude(P_pa):
    # Hypsometric inversion: h = T0/L * [1 - (P/P0)^(1/α)]
    return (T0 / L) * (1.0 - (P_pa / P0) ** (1.0 / ALPHA))


def quat_to_R(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def quat_to_roll_pitch(q):
    rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)
    return float(rpy[0]), float(rpy[1])


class AltitudeFusionNode(Node):

    def __init__(self):
        super().__init__('altitude_fusion_node')

        self.declare_parameter('ground_pressure_pa', 94501.69)
        self.declare_parameter('auto_calibrate_on_arm', True)

        # EKF-1 (IMU+Baro). Q values re-tuned for 100 Hz IMU.
        self.declare_parameter('ekf1_q_z',    0.02)
        self.declare_parameter('ekf1_q_vz',   0.20)
        self.declare_parameter('ekf1_r_baro', 0.50)   # adaptive floor

        # EKF-2 (Baro+LiDAR, adaptive R).
        self.declare_parameter('ekf2_q_z',    0.01)
        self.declare_parameter('ekf2_q_vz',   0.10)
        self.declare_parameter('ekf2_r_baro', 0.50)

        # ACF weights.
        self.declare_parameter('acf_alpha_base', 0.97)
        self.declare_parameter('acf_alpha_min',  0.70)
        self.declare_parameter('acf_alpha_max',  0.99)

        # IAKF window length in samples. At 100 Hz LiDAR, 50 samples = 0.5 s,
        # which is statistically enough for a reliable empirical variance.
        self.declare_parameter('innov_window_size', 50)

        # Accel bias EMA time constant. Large = slow adaptation.
        self.declare_parameter('accel_bias_tau', 20.0)

        # Adaptive baro-R ring buffer length (samples).
        self.declare_parameter('baro_var_window', 8)

        self.P_ground = self.get_parameter('ground_pressure_pa').value
        self.h_ground = pressure_to_altitude(self.P_ground)

        # --- Stage 1 (EKF-1) state ---
        # Model: x = [z, vz], predict with az from IMU, update with baro.
        q_z1  = self.get_parameter('ekf1_q_z').value
        q_vz1 = self.get_parameter('ekf1_q_vz').value
        self.r_b1_default = self.get_parameter('ekf1_r_baro').value

        self.ekf1_x = np.zeros(2)
        self.ekf1_P = np.eye(2)
        self.Q1 = np.diag([q_z1, q_vz1])
        self.R1 = np.array([[self.r_b1_default]])
        self.H_obs = np.array([[1.0, 0.0]])

        self.h_baro_fused = 0.0

        # Adaptive baro-R: rolling buffer of recent baro altitudes.
        self.baro_var_win = self.get_parameter('baro_var_window').value
        self.baro_alt_buf = []

        # --- Stage 2A (ACF) state ---
        self.acf_alpha_base = self.get_parameter('acf_alpha_base').value
        self.acf_alpha_min  = self.get_parameter('acf_alpha_min').value
        self.acf_alpha_max  = self.get_parameter('acf_alpha_max').value
        self.h_acf = 0.0

        # --- Stage 2B (EKF-2) state ---
        # Same kinematic model as EKF-1, but receives both baro and LiDAR
        # updates. LiDAR R is adapted per-sample based on spec + IAKF window.
        q_z2  = self.get_parameter('ekf2_q_z').value
        q_vz2 = self.get_parameter('ekf2_q_vz').value
        r_b2  = self.get_parameter('ekf2_r_baro').value

        self.ekf2_x = np.zeros(2)
        self.ekf2_P = np.eye(2)
        self.Q2 = np.diag([q_z2, q_vz2])
        self.R2_baro = np.array([[r_b2]])
        self.R_lidar_min = 0.01   # (0.10 m)²
        self.R_lidar_max = 5.00   # clamp for degenerate cases

        self.win_size = self.get_parameter('innov_window_size').value
        self.innov_buf = []

        # --- shared timing / IMU-derived state ---
        self.last_imu_t = None
        self.dt = 0.01   # prior guess before first real dt arrives
        self.az_world = 0.0

        # Slow EMA of vertical accel tracks sensor bias; we subtract it
        # before every predict to avoid a velocity ramp between baro updates.
        self.accel_bias_tau = self.get_parameter('accel_bias_tau').value
        self.az_bias = 0.0

        # Stored every IMU msg, consumed by LiDAR tilt correction.
        self.imu_roll  = 0.0
        self.imu_pitch = 0.0

        self.LIDAR_MIN = 0.10
        self.LIDAR_MAX = 180.0
        self.lidar_valid = False

        # Auto-calibration state.
        self._auto_calibrate = self.get_parameter('auto_calibrate_on_arm').value
        self._calibrated = False
        self._last_fcu_armed = False
        self._last_P_raw = None

        # --- subscribers ---
        self.sub_imu   = self.create_subscription(
            Imu, '/managed/imu/data', self._cb_imu, MAVROS_QOS)
        self.sub_baro  = self.create_subscription(
            FluidPressure, '/managed/imu/static_pressure', self._cb_baro, MAVROS_QOS)
        self.sub_lidar = self.create_subscription(
            LaserScan, '/managed/lidar_down', self._cb_lidar, MAVROS_QOS)
        # FCU state — arm-edge detection only.
        self.sub_state = self.create_subscription(
            State, '/mavros/state', self._cb_state, MAVROS_QOS)

        # --- publishers ---
        self.pub_baro_alt = self.create_publisher(Float64, '/fusion/stage1/baro_altitude', 10)
        self.pub_baro_vel = self.create_publisher(Float64, '/fusion/stage1/baro_velocity', 10)
        self.pub_acf_alt  = self.create_publisher(Float64, '/fusion/stage2a/acf_altitude', 10)
        self.pub_ekf2_alt = self.create_publisher(Float64, '/fusion/stage2b/ekf2_altitude', 10)
        self.pub_ekf2_vel = self.create_publisher(Float64, '/fusion/stage2b/ekf2_velocity', 10)
        self.pub_diag     = self.create_publisher(Float64MultiArray, '/fusion/diagnostics', 10)

        self.get_logger().info(
            f"[fusion] up. "
            f"P_ground={self.P_ground:.2f}Pa h_ground={self.h_ground:.2f}m "
            f"iakf_win={self.win_size} bias_tau={self.accel_bias_tau:.0f}s"
        )

    # --- IMU callback (~100 Hz) -------------------------------------------

    def _cb_imu(self, msg):
        # 1) Compute dt, guard stale/zero.
        # 2) Rotate body accel -> world, subtract gravity.
        # 3) Update accel bias EMA, subtract.
        # 4) Store roll/pitch for later LiDAR tilt correction.
        # 5) Predict EKF-1 and EKF-2.
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_imu_t is None:
            self.last_imu_t = now
            return
        dt = now - self.last_imu_t
        if dt <= 0.0 or dt > 0.5:
            self.last_imu_t = now
            return
        self.dt = dt
        self.last_imu_t = now

        # IMU message is body-frame, uncompensated. Rotate to ENU and remove g.
        a_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        a_world = quat_to_R(msg.orientation) @ a_body
        az_raw = a_world[2] - G

        # Slow low-pass over az tracks the long-run bias. Without this we get
        # a velocity drift in both filters between baro updates.
        alpha_bias = dt / (self.accel_bias_tau + dt)
        self.az_bias += alpha_bias * (az_raw - self.az_bias)
        az = az_raw - self.az_bias
        self.az_world = az

        self.imu_roll, self.imu_pitch = quat_to_roll_pitch(msg.orientation)

        self._ekf1_predict(dt, az)
        self._ekf2_predict(dt, az)

    # --- Baro callback (~4 Hz) --------------------------------------------

    def _cb_baro(self, msg):
        P = msg.fluid_pressure
        self._last_P_raw = P   # cached for auto-calibration on arm edge

        h_rel = pressure_to_altitude(P) - self.h_ground

        # Adaptive R1: inflate based on recent sample variance, floor at default.
        # Prevents over-trusting the baro during gust / downwash periods.
        self.baro_alt_buf.append(h_rel)
        if len(self.baro_alt_buf) > self.baro_var_win:
            self.baro_alt_buf.pop(0)
        if len(self.baro_alt_buf) >= 4:
            baro_var = float(np.var(self.baro_alt_buf))
            self.R1 = np.array([[max(baro_var, self.r_b1_default)]])

        self._ekf1_update(h_rel)
        self.h_baro_fused = float(self.ekf1_x[0])

        self._pub(self.pub_baro_alt, self.h_baro_fused)
        self._pub(self.pub_baro_vel, float(self.ekf1_x[1]))

        # Stage-2B also consumes baro altitude as one of two measurement sources.
        self._ekf2_update(h_rel, self.R2_baro, label='baro')

    # --- FCU state callback (arm-edge auto-calibration) -------------------

    def _cb_state(self, msg):
        armed_now = bool(msg.armed)
        first_arm_edge = armed_now and not self._last_fcu_armed
        self._last_fcu_armed = armed_now

        if (not self._auto_calibrate
                or self._calibrated
                or not first_arm_edge
                or self._last_P_raw is None):
            return

        P_new = float(self._last_P_raw)
        # Reject if the delta is inconsistent with being on the ground —
        # protects against mid-flight node restart + re-arm.
        if abs(P_new - self.P_ground) > 600.0:
            self.get_logger().warn(
                f"[fusion] auto-cal REJECTED: ΔP={P_new - self.P_ground:+.1f}Pa "
                f"(> 600Pa, likely airborne); keeping P_ground={self.P_ground:.2f}")
            self._calibrated = True
            return

        P_old = self.P_ground
        self.P_ground = P_new
        self.h_ground = pressure_to_altitude(self.P_ground)
        self._calibrated = True
        self.get_logger().info(
            f"[fusion] auto-cal on arm: P {P_old:.2f} -> {self.P_ground:.2f} Pa "
            f"(Δ={self.P_ground - P_old:+.2f})")

    # --- LiDAR callback (~100 Hz) -----------------------------------------

    def _cb_lidar(self, msg):
        if not msg.ranges:
            return
        r = msg.ranges[0]

        self.lidar_valid = (np.isfinite(r)
                            and self.LIDAR_MIN <= r <= self.LIDAR_MAX)
        if not self.lidar_valid:
            return

        # Tilt correction: h = r_slant · cos(roll) · cos(pitch). Exact
        # for rigid tilt up to ~30°; degenerate above ~60° so we clamp.
        cos_tilt = float(np.clip(np.cos(self.imu_roll) * np.cos(self.imu_pitch),
                                 0.5, 1.0))
        h_lidar = float(r) * cos_tilt

        self._acf_update(h_lidar)
        self._pub(self.pub_acf_alt, self.h_acf)

        R_lidar = self._compute_R_lidar(h_lidar)
        self._ekf2_update(h_lidar, np.array([[R_lidar]]), label='lidar')

        self._pub(self.pub_ekf2_alt, float(self.ekf2_x[0]))
        self._pub(self.pub_ekf2_vel, float(self.ekf2_x[1]))

        # Diagnostics packet (consumer must know layout):
        #   0 raw_lidar        1 tilt_corrected    2 baro_stage1
        #   3 acf              4 ekf2_alt          5 ekf2_vz
        #   6 adaptive_R       7 1σ uncertainty    8 cos_tilt
        #   9 az_bias
        diag = Float64MultiArray()
        diag.data = [
            float(r),
            h_lidar,
            self.h_baro_fused,
            self.h_acf,
            float(self.ekf2_x[0]),
            float(self.ekf2_x[1]),
            R_lidar,
            float(np.sqrt(self.ekf2_P[0, 0])),
            cos_tilt,
            self.az_bias,
        ]
        self.pub_diag.publish(diag)

    # --- Stage 1: EKF-1 ---------------------------------------------------
    # State: x = [z, vz]. Const-accel kinematics. Baro is the only measurement.

    def _ekf1_predict(self, dt, az):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([0.5 * dt ** 2, dt])
        self.ekf1_x = F @ self.ekf1_x + B * az
        self.ekf1_P = F @ self.ekf1_P @ F.T + self.Q1

    def _ekf1_update(self, h_baro):
        # Joseph form keeps P symmetric PSD under finite-precision arithmetic.
        H = self.H_obs
        innov = h_baro - (H @ self.ekf1_x)[0]
        S = (H @ self.ekf1_P @ H.T + self.R1)[0, 0]
        K = (self.ekf1_P @ H.T).flatten() / S
        self.ekf1_x = self.ekf1_x + K * innov
        IKH = np.eye(2) - np.outer(K, H)
        self.ekf1_P = IKH @ self.ekf1_P @ IKH.T + np.outer(K, K) * self.R1[0, 0]

    # --- Stage 2A: Adaptive Complementary Filter --------------------------

    def _acf_update(self, h_lidar):
        # h_acf = α · (h_acf_prev + vz·dt) + (1-α) · h_lidar
        # α adapts on two axes:
        #   - altitude band: below 10 m trust LiDAR more (spec is ±10 cm),
        #     above 10 m trust it less (spec is 1%, error grows with h).
        #   - innovation gate: if LiDAR vs prediction diverges >1 m, inflate α.
        vz = float(self.ekf2_x[1])
        h_pred = self.h_acf + vz * self.dt

        if h_lidar < 10.0:
            t = h_lidar / 10.0
            alpha = self.acf_alpha_min + t * (self.acf_alpha_base - self.acf_alpha_min)
        else:
            sigma_lidar = 0.01 * h_lidar
            scale = min((sigma_lidar - 0.10) / 1.70, 1.0)
            alpha = self.acf_alpha_base + scale * (self.acf_alpha_max - self.acf_alpha_base)

        if abs(h_lidar - h_pred) > 1.0:
            alpha = min(alpha + 0.04, self.acf_alpha_max)

        self.h_acf = alpha * h_pred + (1.0 - alpha) * h_lidar

    # --- Stage 2B: EKF-2 with IAKF ----------------------------------------

    def _ekf2_predict(self, dt, az):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([0.5 * dt ** 2, dt])
        self.ekf2_x = F @ self.ekf2_x + B * az
        self.ekf2_P = F @ self.ekf2_P @ F.T + self.Q2

    def _compute_R_lidar(self, h):
        # Spec-based baseline:
        #   h < 10 m : σ = 0.10 m -> R = 0.01
        #   h >= 10 m: σ = 0.01·h -> R = (0.01·h)²
        sigma = 0.10 if h < 10.0 else 0.01 * h
        R_spec = sigma ** 2

        # IAKF: if empirical innovation variance exceeds theoretical, inflate R.
        # Guards against overconfidence when the measurement process is noisier
        # than the spec suggests (rough terrain, foliage, etc).
        if len(self.innov_buf) >= 5:
            S_emp = float(np.var(self.innov_buf))
            S_th  = float((self.H_obs @ self.ekf2_P @ self.H_obs.T)[0, 0]) + R_spec
            if S_emp > S_th:
                R_spec = S_emp * 1.5

        return float(np.clip(R_spec, self.R_lidar_min, self.R_lidar_max))

    def _ekf2_update(self, z_meas, R, label=''):
        H = self.H_obs
        innov = z_meas - (H @ self.ekf2_x)[0]

        # Only LiDAR innovations go in the IAKF window. Including baro would
        # mix two different measurement processes.
        if label == 'lidar':
            self.innov_buf.append(innov)
            if len(self.innov_buf) > self.win_size:
                self.innov_buf.pop(0)

        S = (H @ self.ekf2_P @ H.T + R)[0, 0]
        K = (self.ekf2_P @ H.T).flatten() / S
        self.ekf2_x = self.ekf2_x + K * innov
        IKH = np.eye(2) - np.outer(K, H)
        self.ekf2_P = IKH @ self.ekf2_P @ IKH.T + np.outer(K, K) * R[0, 0]

    # --- tiny publish helper ----------------------------------------------

    @staticmethod
    def _pub(publisher, value):
        msg = Float64()
        msg.data = value
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeFusionNode()
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
