# RC car Gazebo screenshots (SIMULATION ONLY)

Captured 2026-11-22, off-screen (gz sim --headless-rendering, software rendering) with static
camera sensors in a screenshot-only world variant (`app/rc_car/figures/make_screenshot_world.py`).
They are figures, not results. They do not go into `paper/` unless the user and the manuscript agent
decide so (STOP 4a review, section 4).

| File | What it shows |
|---|---|
| `sim_gazebo_rccar_overview_baseline-rpp.png` | The whole tracked area: blue = the 10 x 10 m OptiTrack area (PLACEHOLDER size), orange = the 1 m geofence, black = the smooth figure-eight reference (lemniscate, half-width 3.5 m); the car on the right lobe |
| `sim_gazebo_rccar_closeup_baseline-rpp.png` | 3/4 view of the Reely NEW1 model near the centre crossing: blue chassis box, black wheels, green box = the KV260 payload (PLACEHOLDER 0.5 kg) |
| `sim_gazebo_rccar_midrun_baseline-rpp.png` | Mid-run on the right lobe of the figure-eight |

- **Simulation (Gazebo Harmonic).** The vehicle model is a simplified box-and-cylinder model with
  approximate values (track 0.30 m and full lock 0.356 rad measured approximately; wheelbase from the
  manual; masses PLACEHOLDER).
- **The controller shown is the BASELINE** (Nav2 Regulated Pure Pursuit with our Twist->Ackermann
  converter), not the NMPC: figure-eight at 1.0 m/s, `realistic_guess` actuator and mocap emulation,
  seed 1 (this run: cross-track RMSE 2.0 mm).
- The tracked-area size is a PLACEHOLDER (about 100 m^2; exact sides pending).
- Raw frames: `~/nmpc_sim/logs/rc_car_screenshots_20261122/shoot2/`.
