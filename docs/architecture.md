# Architecture

Reference document for the altitude-fusion pipeline, biome layout, and
disturbance model. Complements the code-level comments in `src/`.

## Altitude fusion pipeline

Three stages running in parallel. Each produces an independent altitude
estimate; downstream consumers pick which one to use.

```
           IMU (100 Hz)  ─┬───────────────────┬──────────────────┐
                          │                   │                  │
                          ▼                   ▼                  ▼
                       EKF-1 predict       ACF predict       EKF-2 predict
                          ▲                   ▲                  ▲
   Baro (4 Hz) ───────────┘                   │                  ├──── Baro (4 Hz)
                                              │                  │
                                              └──── LiDAR ───────┘
                                                   (100 Hz, tilt-corrected)

Outputs:   /fusion/stage1/baro_altitude     (Stage-1 EKF-1 / ASL-ish)
           /fusion/stage2a/acf_altitude     (Stage-2A ACF / AGL)
           /fusion/stage2b/ekf2_altitude    (Stage-2B EKF-2 IAKF / mixed)
```

### Stage 1 — EKF-1 (IMU + Baro)

2-state EKF, `x = [z, vz]`. Predicts from IMU vertical acceleration,
updates from baro-derived altitude. Baro measurement noise R is inflated
adaptively from a small rolling buffer of recent readings, so wind gusts
or motor downwash don't convince the filter baro is tight when it isn't.
Floor at the tuned default.

### Stage 2A — Adaptive Complementary Filter (ACF)

Single-state LiDAR-dominant filter. Prediction is `h + vz*dt` (using
Stage-2B's vz); correction blends in LiDAR with weight `(1-α)`. α adapts
in two places:

- **Altitude band**. Below 10 m, LiDAR spec is ±10 cm, so trust it more.
  Above 10 m, spec degrades to ±1%, so trust it less.
- **Innovation gate**. If the LiDAR reading disagrees with the prediction
  by more than 1 m, bump α toward its max — rejects transient outliers.

Key property: **ACF never consumes baro**. Baro drift cannot affect it.
This is why ACF is robust under the `drift` scenario.

### Stage 2B — EKF-2 with IAKF

Same 2-state kinematic model as EKF-1, but with two measurement channels:
baro (~4 Hz) and LiDAR (~100 Hz). LiDAR R adapts per-sample:

1. **Spec-based baseline**. `σ = 0.10 m` if h<10 m, else `σ = 0.01·h`.
2. **IAKF inflation**. If the empirical innovation variance over the last
   50 LiDAR samples exceeds the theoretical S = HPH + R, inflate R to
   `1.5·S_emp`. Clamps to `[0.01, 5.0]`.

This rejects rough-terrain noise (variance-triggered) but not constant
bias (needs a separate magnitude gate, scoped out).

### Tilt correction

LiDAR points straight down in body frame, so during pitch/roll it measures
a slant range. Every IMU callback stores the current roll/pitch; every
LiDAR callback projects: `h_vertical = r_slant · cos(roll) · cos(pitch)`.
Clamps `cos_tilt` to 0.5 as a safety floor. Degrades gracefully above 60°.

### Accelerometer bias EMA

Slow first-order low-pass (τ ≈ 20 s) tracks the long-run mean of vertical
accel. Without this, small bias/gravity-residual errors integrate into a
velocity ramp between baro updates in both filters. Bias is subtracted
before each predict step.

### Auto-calibration on first arm

Fresh SITL sessions land at slightly different ambient pressures each
run, which used to produce a 3-6 m constant bias on Stage-1 altitude.
On the first disarmed→armed edge, we capture the current raw baro
reading as the ground-pressure reference. Rejected if the delta is >
600 Pa (≈ 60 m) — that means we restarted mid-flight, so using the
current pressure would calibrate to whatever altitude we were already
at. Only fires once per session.

## Biomes

Three circular regions (35 m radius) in a 200 m × 300 m Gazebo world.

| Biome | Centre (E, N) | Orbit altitude | Ground |
|-------|---------------|----------------|--------|
| CITY | (0, -100) | 70 m | Elevated ~13 m (buildings) |
| VEGETATION | (90, 0) | 30 m | Flat (trees, little AGL impact) |
| TERRAIN | (0, 150) | 75 m | Rolling hills, ~22 m average |

Transit legs between biomes all happen at 80 m absolute. This unifies
the altitude reference, so any altitude variation observed during
transits has to come from LiDAR picking up different ground heights
— not from the drone changing altitude.

## Disturbance model

`sensor_rate_manager.py` injects synthetic disturbances at the
republish stage, before anything else in the pipeline sees them. Four
modes plus NONE:

- **WIND** — random walk + Gaussian on baro pressure. Random walk
  captures slow gust correlations; Gaussian is fast turbulence.
  Parameters: `wind_baro_rw_sigma_pa`, `wind_baro_gauss_sigma_pa`,
  `wind_baro_rw_decay`.
- **LIDAR_DROPOUT** — stochastic bursts (25 samples default) where
  LiDAR returns `inf`, plus single-sample spikes with random sign and
  magnitude. Parameters: `lidar_dropout_burst_prob`, `_burst_len`,
  `lidar_spike_prob`, `_spike_magnitude_m`.
- **BARO_DRIFT** — linear pressure ramp + slow oscillation. The
  paper's headline stress: drift is a *bias*, not a *variance*, so
  IAKF's variance adaptation doesn't detect it. Parameters:
  `baro_drift_rate_pa_per_s`, `_oscillation_hz`, `_osc_amp_pa`.
- **COMBINED** — all three simultaneously, at reduced intensity.

Two-tier arming: `disturbance_mode` sets the mode; `disturbance_active`
flips it on/off. Scenario scripts configure mode while disarmed, then
flip `disturbance_active=True` once the drone is past takeoff — so the
climb isn't corrupted before the controller has authority.

## Failsafe pipeline

`failsafe_node.py` runs a 10 Hz FSM across three independent failure
domains:

1. **LiDAR integrity**. Three-tier escalation: invalid <0.5s is ignored
   (noise), 0.5-3s is LIDAR_DEGRADED (informational), 3-15s is
   LIDAR_FAILED (takes control, executes reversed-heading retreat),
   ≥15s forces BATTERY_LOW path (retrospective landing search).
2. **Battery**. Below 25% triggers BATTERY_LOW immediately.
3. **Variance ring buffer**. During TRANSIT/ORBIT on-target, samples
   `(x, y, variance, mean_range, t)` at 1 Hz into a 600-slot buffer.
   Used at BATTERY_LOW time to score past locations for safe landing.

When active (state ≥ LIDAR_FAILED), failsafe publishes `PositionTarget`
messages on `/failsafe/command` at 20 Hz, and mission_node forwards
them to `/mavros/setpoint_raw/local`. The MAVROS setpoint topic has a
single writer (mission_node), ever — no contention.

## Dead-reckoning

Horizontal pose is integrated by mission_node from the *commanded*
velocity + heading (intent-integral), not from IMU. This is published
on `/mission/dead_reckoned_pose` for the failsafe variance buffer and
for plotting. The gap between this and MAVROS pose is the GPS-denied
story: it's visible in the bird's-eye panel of every scenario plot.
