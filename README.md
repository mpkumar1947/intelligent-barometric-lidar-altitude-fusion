# GPS-Denied UAV Altitude Fusion

Three-stage adaptive altitude fusion pipeline for a GPS-denied quadrotor.
IMU + Barometer + downward LiDAR (Benewake TF03-180), simulated in
ArduPilot SITL + Gazebo with MAVROS on ROS 2.

EE798T UAV Communication course project, IIT Kanpur.
Manemoni Pavan Kumar (230630), M Shiva Teja (230610), Marripudi Akhil (230638).

---

## What it does

The drone flies an 18-leg mission over three contrasting biomes — CITY
(urban, 70 m orbit over elevated ground), VEGETATION (flat, 30 m orbit
over trees), TERRAIN (hilly, 75 m orbit). Horizontal motion is open-loop
dead-reckoning on commanded `(heading, speed, duration)`; altitude is
closed-loop on a fused estimate from three stages running in parallel:

| Stage | Sensors | What it estimates | Strength |
|-------|---------|-------------------|----------|
| 1 — EKF-1 | IMU + Baro | ASL altitude | Clean tracking over flat terrain |
| 2A — ACF | IMU + LiDAR | AGL altitude (LiDAR-dominant) | Immune to baro faults |
| 2B — EKF-2 IAKF | IMU + Baro + LiDAR | Mixed, adaptive R | Balanced performer |

Stage 2B uses an innovation-adaptive Kalman filter (IAKF) that inflates
the LiDAR measurement noise when the innovation variance grows. This
rejects rough-terrain noise, but **does not** defend against slow bias
drift on the baro — which is itself a useful negative result.

A separate failsafe node monitors LiDAR integrity, battery, and a
retrospective landing-search buffer (variance-scored past waypoints).

---

## Results summary

Five disturbance scenarios (nominal, wind, drift, dropout, combined) all
run with Gazebo ground-truth pose for RMSE. Key findings:

- **Stage-1 is catastrophic under baro drift** (104 m ALL RMSE) — pure
  IMU+Baro fusion isn't sustainable without a second reference.
- **Stage-2A ACF survives drift with barely a scratch** — VEG RMSE
  stays at 0.79 m while Stage-1 collapses to 77 m on the same run,
  because ACF never sees the baro.
- **Stage-2B EKF-2 degrades partially under drift** (34 m ALL) — IAKF's
  variance-based adaptation doesn't detect constant bias, an honest
  limitation that's quantified here for the first time in this context.
- **The large ACF "errors" over CITY and TERRAIN biomes are not filter
  errors** — they're the AGL/ASL frame difference over elevated ground.
  VEGETATION (flat) is the control biome: all three stages agree to
  within 0.6 m there, confirming filter correctness.

Full RMSE table: see `results/rmse_table.tex` or run
`python3 scripts/evaluation/compute_rmse_table.py`.

---

## Repo layout

```
project/
├── README.md
├── requirements.txt
├── .gitignore
├── LICENSE
├── src/
│   ├── mission_node.py              # mission FSM, dead-reckoning, 20 Hz sp stream
│   ├── altitude_fusion_node.py      # Stage 1 / 2A / 2B fusion
│   ├── failsafe_node.py             # LiDAR/battery monitor, retrospective landing
│   ├── sensor_rate_manager.py       # rate control + disturbance injection
│   ├── gz_pose_bridge.py            # Gazebo ground-truth bridge (Sprint C)
│   └── inspect_gz_transforms.py     # one-shot diagnostic
├── scripts/
│   ├── scenarios/
│   │   ├── _common.sh               # shared bash helpers
│   │   ├── run_nominal.sh
│   │   ├── run_wind.sh
│   │   ├── run_drift.sh
│   │   ├── run_dropout.sh
│   │   ├── run_combined.sh
│   │   └── run_all.sh
│   └── evaluation/
│       ├── compute_rmse_table.py    # per-biome RMSE markdown/LaTeX
│       └── smoke_test_gz_bridge.sh  # end-to-end bridge sanity check
├── docs/
│   └── architecture.md              # pipeline + biome + disturbance model
├── results/
│   └── rmse_table.tex               # RMSE table
└── figures/                         # 10 final scenario plots
```

---

## Running it

### Dependencies

- ROS 2 Jazzy (or Humble; paths in scripts assume Jazzy)
- ArduPilot SITL built with `gazebo-iris` model
- Gazebo Harmonic (comes with Jazzy) + `ros-jazzy-ros-gz-bridge`
- Python 3.10+ and `pip install -r requirements.txt`

### Session bring-up (terminal by terminal)

| Term | What |
|------|------|
| 1 | `gz sim -v4 -r worlds/combined_drone_world.sdf` |
| 2 | `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console` |
| 3 | `ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555` |
| 4 | `ros2 run ros_gz_bridge parameter_bridge   /lidar_down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan` |
| 5 | `ros2 run ros_gz_bridge parameter_bridge /world/combined_drone_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V` |
| 6 | `python3 src/sensor_rate_manager.py` + `src/altitude_fusion_node.py` + `src/failsafe_node.py` |
| 7 | `python3 src/gz_pose_bridge.py` (wait for `DETECTION LOCKED`) |
| 8 | `bash scripts/scenarios/run_nominal.sh` (or any other scenario) |

`scripts/evaluation/smoke_test_gz_bridge.sh` verifies terminals 1, 5, 7
are working before you start runs.

### Running a scenario

From the project root (terminal 8):

```bash
bash scripts/scenarios/run_nominal.sh
# ~9 minutes. Plot + log + pickle land in outputs/ and logs/.
```

All 5 back-to-back:

```bash
bash scripts/scenarios/run_all.sh
# ~50 minutes including 20s settling between scenarios.
```

### Computing the RMSE table

After all 5 scenarios have pickles in `logs/`:

```bash
python3 scripts/evaluation/compute_rmse_table.py
# markdown, default output

python3 scripts/evaluation/compute_rmse_table.py --latex > results/rmse_table.tex
# LaTeX for the paper
```

---

## Known limitations / not claimed

- Variance-based adaptive R doesn't detect constant-bias drift on the
  baro (see `drift` scenario result). A chi-square magnitude gate
  would complement this — scoped out of this project.
- Stage-2A ACF reports AGL, Stage-1 reports ASL, Stage-2B reports a
  mixed estimate anchored to baro. Over non-flat terrain these
  legitimately disagree by tens of metres — this is physics, not
  filter error.
- Horizontal position is open-loop dead-reckoning. The drift between
  that and MAVROS EKF3 pose *is* the GPS-denied story; don't expect it
  to converge.
- Dropout and combined runs land early when the failsafe trips — this
  is correct safety behaviour but means those rows have `---` entries
  for biomes the drone never reached. The RMSE script handles this
  gracefully (insufficient-sample flag).

---
## License

MIT — see `LICENSE`.
