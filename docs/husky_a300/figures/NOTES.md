# Husky A300 Gazebo screenshots (SIMULATION ONLY)

Captured 2026-11-22, off-screen (gz sim --headless-rendering, software rendering) with static
camera sensors in a screenshot-only copy of Clearpath's `warehouse` world
(`app/rc_car/figures/make_screenshot_world.py`; real-time factor 0.25 in the world file). They are
figures, not results. They do not go into `paper/` unless the user and the manuscript agent decide so
(STOP 4a review, section 4).

| File | What it shows |
|---|---|
| `sim_gazebo_husky_overview_sil-pso.png` | The A300 in the warehouse on the first loop of the SIL figure-eight (two 1 m circles, black, drawn from the spawn pose (-4, 4, 0)) |
| `sim_gazebo_husky_closeup_sil-pso.png` | 3/4 view of the A300 at the start pose |
| `sim_gazebo_husky_midrun_sil-pso.png` | Mid-run on the first loop |

- **Simulation (Gazebo Harmonic, Clearpath A300 model).**
- **Controller: the unicycle PSO-NMPC as a Python SIL emulation of the solver (`app/husky/sil`,
  10x40, plain predictor, seed 1), NOT the KV260.**
- **The robot tracks its own odometry** (the generated EKF: wheel odometry + IMU), not the true pose.
  The black reference is drawn at the true start pose. The first loop matches it (centre about
  (-3.9, 5.0), radius about 1.05 m in the ground truth); by its end the odometry has drifted, so the
  second loop lies about 2 m to the left of the drawn one. The overview and mid-run images are
  therefore taken on the first loop; the drift is the SIL's real behaviour, not hidden.
- Run log: `~/nmpc_sim/logs/husky_screenshots_20261122/sil/` (raw frames in `frames/`).
