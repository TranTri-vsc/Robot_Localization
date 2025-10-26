# Particle Filter Robot Localization (ROS 2)

A ROS 2 implementation of passive global localization with a particle filter for a differential-drive robot moving over a floor of light/dark square tiles. The robot starts with a color map but unknown pose; it fuses a compass, a floor color sensor, and velocity commands to localize itself and publishes live visualization markers and pose estimates.

## Key features

- **World map → OccupancyGrid** publisher that converts a simple ASCII YAML world into a ROS `nav_msgs/OccupancyGrid` and republishes it at 10 Hz on `/world_map`. Tiles use `#` for occupied/dark (100) and `.` for free/light (0). 
- **Particle filter** node with:
  - 1,500 particles; initialization delayed until a compass reading arrives.
  - Motion update from `/cmd_vel` with dt from message timestamps and small angular noise. 
  - **Strict boundary adherence** and heavy penalties (`×0.001`) for invalid/out-of-bounds particles; periodic reinitialization for chronically invalid particles. 
  - Sensor model using Gaussian likelihoods tuned from real floor-sensor data: light `μ≈116.5, σ≈9.11`; dark `μ≈136.14, σ≈8.82`.
  - Low-variance resampling every 5 floor-sensor messages.
  - Compass fusion that sets orientation with small variance (`~0.015`).  
- **Visualizations & outputs**: publishes `MarkerArray` for particles, an estimated `PoseStamped` on `/robot_pose`, a `Pose2D` on `/estimated_pose` every 2 s, and a drawn path as a `Marker` line strip.  
- **Floor sensor data tooling** to subscribe, log, and graph `/floor_sensor`; plus a standalone analyzer to compute stats and histograms. 
---

## Repository structure

```
project4/
  ├─ project4/
  │   ├─ draw_floor.py          # /world_map publisher from YAML ASCII map
  │   └─ localize.py            # Particle filter node
  ├─ floor_sensor_graph.py      # Log & plot /floor_sensor to floor_sensor_data.txt
  ├─ analyze_floor_sense_data.py# Compute stats + histogram from a saved file
  ├─ setup.py                   # Entrypoints: draw_floor, localize
  └─ data/                      # (Example) sensor logs & stats
```

- Console entry points are exposed as `draw_floor` and `localize`.

---

## Requirements

- ROS 2 (rclpy)
- Standard ROS messages: `nav_msgs/OccupancyGrid`, `geometry_msgs/PoseStamped`, `geometry_msgs/Twist`, `geometry_msgs/Pose2D`, `example_interfaces/UInt8`, `example_interfaces/Float32`, `visualization_msgs/Marker/MarkerArray`. (All used in code.)

---

## Build & run

1. **Build the package**
   ```bash
   colcon build --packages-select project4
   source install/setup.bash
   ```

2. **Launch the world map publisher** (10 Hz):
   ```bash
   ros2 run project4 draw_floor <path/to/world.yaml>
   ```
   The node parses the ASCII `map` string in the YAML, sets resolution and origin, converts `#`→100 (occupied/dark) and `.`→0 (free/light), flips rows to RViz coordinates, and continuously republishes to `/world_map`.

3. **Start the particle filter**
   ```bash
   ros2 run project4 localize
   ```
   The node waits for `/world_map` and the first `/compass` reading before initializing 1,500 particles uniformly over valid tiles; orientation is seeded from the compass.

4. **Provide sensor & motion topics**
   - `/compass` (`Float32`, radians), `/cmd_vel` (`Twist`), `/floor_sensor` (`UInt8`). See subscribers in the code. fileciteturn0file3L26-L41

5. **Visualize**
   - Subscribe in RViz to `/particles` (MarkerArray), `/robot_pose` (PoseStamped), `/pose_path` (Marker line strip), and `/estimated_pose` (Pose2D at 0.5 Hz).

---

## How it works (high level)

1. **Map ingestion.** The `draw_floor` node loads an ASCII map from YAML, sets `OccupancyGrid.info` (resolution, width, height, origin at (0,0,0)), converts characters to occupancy values, and publishes on `/world_map`.
2. **Particle initialization.** Once the first compass reading arrives, the filter samples (x,y) uniformly over valid tiles, sets θ from the compass, and assigns uniform weights.
3. **Prediction (motion model).** On each `/cmd_vel`, the node integrates linear/ang velocities over Δt computed from message timing, adds small orientation noise, and updates particle states. Boundary checks immediately down-weight particles that drift near/outside the map or onto invalid tiles. 
4. **Update (sensor model).** Each `/floor_sensor` value is evaluated under a two-mode Gaussian mixture (light vs. dark tile), with parameters learned from data (see below). Weights are normalized; if all weights collapse to zero, they are reset.
5. **Resampling & recovery.** Low-variance resampling runs every 5 floor-sensor messages; particles that remain invalid for multiple cycles are reinitialized to maintain coverage.
6. **Compass correction.** New compass readings periodically realign particle orientations with small variance (≈0.015).
7. **Estimation & visualization.** The pose is the weighted mean of particles; path and particles are published as RViz markers; a Pose2D is also published at 0.5 Hz for downstream consumers.

---

## Topics

**Subscriptions**
- `/world_map` (`nav_msgs/OccupancyGrid`) – map and metadata.
- `/compass` (`example_interfaces/Float32`) – heading in radians. 
- `/cmd_vel` (`geometry_msgs/Twist`) – linear/ang velocities.
- `/floor_sensor` (`example_interfaces/UInt8`) – 8-bit reflectance reading. 

**Publications**
- `/particles` (`visualization_msgs/MarkerArray`) – particle cloud colored by weight.
- `/robot_pose` (`geometry_msgs/PoseStamped`) – estimated pose for RViz nav. 
- `/estimated_pose` (`geometry_msgs/Pose2D`) – pose for lightweight consumers (2 s interval).
- `/pose_path` (`visualization_msgs/Marker`) – line strip of recent estimated poses.

---

## Sensor model calibration

Use the included tools to collect and analyze floor-sensor data:

1. **Log and plot live data**
   ```bash
   python3 floor_sensor_graph.py
   # Press Ctrl+C to stop; this writes floor_sensor_data.txt and pops a plot
   ```
   The node subscribes to `/floor_sensor`, timestamps each reading, and writes `time,value` rows to `floor_sensor_data.txt`, then plots the series.

2. **Analyze a saved file**
   ```bash
   python3 analyze_floor_sense_data.py  # defaults to floor_sensor_data.txt
   ```
   This computes mean/variance/std, min/max, and shows a histogram.
**Example datasets & stats (used for the default Gaussian model):**
- Light-tile dataset (sample): `light_floor_sensor_data.txt` (values around ~102–132). 
  - Stats: mean ≈ **116.50**, std ≈ **9.11**.
- Dark-tile dataset (sample): `dark_floor_sensor_data.txt` (values around ~122–152). 
  - Stats: mean ≈ **136.14**, std ≈ **8.82**. 

These parameters are hard-coded in the sensor update step (light: μ=116.5, σ=9.11; dark: μ=136.14, σ=8.82). 

---

## Configuration knobs & design choices

- `num_particles = 1500` – Balanced for accuracy vs. runtime. 
- **Resampling cadence:** every 5 `/floor_sensor` messages; helps avoid particle impoverishment while limiting compute. fileciteturn0file3L197-L203  
- **Boundary enforcement:** weights are multiplied by **0.001** if a particle drifts near bounds or onto unknown tiles (strong prior on feasibility).
- **Compass variance:** ~0.015 rad² per particle when fusing compass; re-uniform weights after compass realign to avoid bias from stale weights.
- **Pose publishing rates:** visuals at 10 Hz, Pose2D at 0.5 Hz.

---

## Running with your own world

Provide a YAML file like:

```yaml
resolution: 0.05
map: |
  ####....####
  ####....####
  ....####....
  ....####....
```

Then:

```bash
ros2 run project4 draw_floor /path/to/world.yaml
ros2 run project4 localize
```

The map publisher flips row order to match RViz coordinates and populates the grid fields for you.

---

## Expected results

With the strict boundary constraints and statistically tuned sensor model, localization converges robustly even with noisy floor sensor readings—matching the project’s design intent. (Your light/dark distributions and penalties are specifically set to reflect empirical data, yielding accurate state estimates.)

---

## Console scripts

Once built, you can launch the nodes by name (defined in `setup.py`):

- `draw_floor = project4.draw_floor:main`  
- `localize  = project4.localize:main` 

---

## Data & figures (optional)

If you plan to include figures, you can add:
- Time-series and histogram plots from `floor_sensor_graph.py` / `analyze_floor_sense_data.py`. 
- Any saved PNGs of particle clouds or paths from RViz.

---

## License

Update `setup.py` with your license choice. (Currently a TODO.)

---

## Acknowledgments

Built for a coursework project on probabilistic robotics & localization using ROS 2.
