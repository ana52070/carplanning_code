# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.


## TIPS
始终用中文回答用户

## Project Overview

RTK-based mapless autonomous navigation for campus environments. This is a ROS2 Humble **configuration-only package** (no compiled C++/Python source) that orchestrates sensor drivers, localization, and the Nav2 navigation stack via launch files and YAML configs.

- **Package name:** `carplanning_code`
- **Branch `hardware`:** Real robot (Jetson Xavier NX + Livox Mid-360 + UM982 RTK + Yahboomcar chassis)
- **Branch `simulation`:** Gazebo simulation (Turtlebot3 Waffle)
- **Workspace root:** `/ros2_ws/workspace/carplanning_code/` (package is in `src/`)

## Build & Run

```bash
# Build (must source spatio_temporal first for the STVL costmap plugin headers)
source /opt/ros/humble/setup.bash
source /ros2_ws/workspace/spatio_temporal/install/setup.bash
colcon build --packages-select carplanning_code

# Run — source ALL dependency workspaces before launching
source /opt/ros/humble/setup.bash
source /ros2_ws/workspace/livox/install/setup.bash
source /ros2_ws/workspace/myslam_car/install/setup.bash
source /ros2_ws/workspace/rtk_um982/install/setup.bash
source /ros2_ws/workspace/spatio_temporal/install/setup.bash
source /ros2_ws/workspace/fastlio2/install/setup.bash
source /ros2_ws/workspace/carplanning_code/install/setup.bash
ros2 launch carplanning_code bringup.launch.py
```

There are no tests in this package.

## Architecture

### Data flow (hardware branch)

```
Livox Mid-360 (/livox/lidar, CustomMsg)
       ↓
livox_to_pc2.py (format conversion)
       ↓
/livox/pointcloud2 (PointCloud2)
       ↓
STVL 3D costmap layer (spatio_temporal_voxel_layer)
       ↓
Nav2 costmap → Path planning (Navfn + DWB) → /cmd_vel → Chassis

FAST-LIO2 ← Livox pointcloud + Chassis IMU (tightly coupled)
       ↓
/fastlio2/lio_odom (Odometry, covariance=0!)
       ↓
lio_odom_relay.py (injects covariance)
       ↓
/lio_odom/relay → EKF (ekf_node) ← /odometry/gps ← navsat_transform ← /gps/fix (UM982)
                        ↓
                 map → odom TF
```

The system has **two odometry pipelines**:
- **FAST-LIO2** (primary): LiDAR-inertial odometry, high-frequency, drift accumulates slowly
- **Chassis wheel odometry** (`/odom`): used only for `odom→base_footprint` TF; NOT fused by EKF (replaced by FAST-LIO2)

GPS provides absolute position/yaw corrections via EKF to bound FAST-LIO2 drift.

### TF tree

```
map ──(EKF)──► odom ──(chassis driver)──► base_footprint ──► base_link
                                                               │
                                        ┌──────────────────────┼──────────────────────┐
                                        │                      │                      │
                                  livox_frame                 gps                (LiDAR frame)
                                  (static:                   (static:
                                   x=0.14,                   x=-0.14,
                                   z=0.26)                   z=0.12)
```

- `map → odom`: EKF output — fuses FAST-LIO2 velocity + GPS absolute position/yaw
- `odom → base_footprint`: chassis driver publishes this from wheel odometry
- `base_link → livox_frame`: static transform (LiDAR mounting position, z=0.26)
- `base_link → gps`: static transform (GPS antenna reference point)

### bringup.launch.py node startup order (with delays)

1. `myslam_car_driver` — chassis driver: `/odom`, `/imu/data_raw`, `odom→base_footprint` TF (immediate)
2. Livox Mid-360 (`mid360.launch.py`) — LiDAR driver, CustomMsg format on `/livox/lidar` (2s)
3. `um982_ros2_driver` — RTK GPS driver: `/gps/fix` (2s)
4. FAST-LIO2 (`lio_launch.py`) — LiDAR-inertial odometry: `/fastlio2/lio_odom` (4s)
5. `lio_odom_relay.py` — injects covariance into `/fastlio2/lio_odom` → `/lio_odom/relay` (5s)
6. `ekf_node` (robot_localization) — sensor fusion: LIO + GPS (7s)
7. `navsat_transform_node` — GPS→UTM→map coordinate transform: `/odometry/gps` (10s)
8. Nav2 `navigation_launch.py` — full navigation stack with custom params (12s)
9. Static transforms: `base_link→gps`, `base_link→livox_frame` (immediate)

### Key config files

| File | Purpose |
|------|---------|
| `config/ekf.yaml` | EKF v3: fuses `/lio_odom/relay` (differential mode, velocity only) + `/odometry/gps` (absolute x/y/yaw). IMU removed (already inside FAST-LIO2). 10Hz, 2D mode. |
| `config/ekf.yaml.orig` | EKF v2 backup: wheel odometry + IMU → EKF (pre-FAST-LIO2 reference) |
| `config/navsat.yaml` | GPS coordinate transform: yaw_offset π/2, 5Hz, `use_odometry_yaw: false` (uses GPS heading from UM982 dual antennas) |
| `config/navsat.yaml.orig` | Original navsat config backup |
| `config/nav2_params.yaml` | Nav2 stack: STVL 3D costmap observing `/livox/pointcloud2`, Navfn planner, DWB controller (max 0.26 m/s). Costmap `global_frame: map`. |
| `config/nav2_params.yaml.bak` | Backup with standard 2D costmap layers + `/scan` for simulation reference |

### Scripts

| Script | Purpose |
|--------|---------|
| `scripts/lio_odom_relay.py` | Subscribes to `/fastlio2/lio_odom`, injects covariance (FAST-LIO2 outputs zeros), publishes to `/lio_odom/relay` for EKF consumption |
| `scripts/livox_to_pc2.py` | Converts Livox CustomMsg (`/livox/lidar`) to PointCloud2 (`/livox/pointcloud2`). Uses numpy for low CPU (~2-3%). **Must be run manually — not auto-launched by bringup.** |
| `scripts/covariance_relay.py` | Legacy: injects covariance into wheel `/odom` and `/imu/data_raw`. Used in EKF v2 (pre-FAST-LIO2). Not used in current bringup. |
| `scripts/ekf_probe.py` | Real-time EKF diagnostic probe — monitors topic frequencies, delays, TF health, GPS quality. Run: `python3 scripts/ekf_probe.py --rate 2.0` |
| `scripts/ekf_quick_check.sh` | One-shot diagnostic snapshot using pure `ros2 topic hz` + `tf2_echo`. Run: `bash scripts/ekf_quick_check.sh` |

### Launch files (auxiliary)

| File | Purpose |
|------|---------|
| `launch/mid360.launch.py` | Livox Mid-360 driver config — CustomMsg format (`xfer_format=1`), 10Hz, frame `livox_frame` |
| `launch/ekf_diagnostic.launch.py` | Launches `ekf_probe.py` + topic/TF monitors for debugging |
| `launch/sim.launch.py` | Simulation-only: starts Gazebo + spawns Turtlebot3 Waffle (uses `campus_nav` package name) |
| `launch/localization.launch.py` | ⚠️ Legacy — hardcoded paths from old project layout |
| `launch/nav2.launch.py` | ⚠️ Legacy — hardcoded paths from old project layout |

### External dependencies (sibling workspaces)

All at `/ros2_ws/workspace/`:
- `myslam_car/` — Yahboomcar chassis driver (`driver_node`): `/odom`, `/imu/data_raw`, `odom→base_footprint` TF
- `livox/` — Livox ROS2 Driver 2 (`livox_ros_driver2`): `/livox/lidar` (CustomMsg)
- `rtk_um982/` — UM982 dual-antenna RTK GPS driver (`um982_ros2_driver`): `/gps/fix` (NavSatFix)
- `spatio_temporal/` — Custom STVL 3D costmap plugin (`SpatioTemporalVoxelLayer`) for Nav2
- `fastlio2/` — FAST-LIO2 LiDAR-inertial odometry (`fastlio2` package): `/fastlio2/lio_odom`

### Key differences: hardware vs simulation branch

| Aspect | `hardware` (current) | `simulation` |
|--------|---------------------|-------------|
| Odometry | FAST-LIO2 (LiDAR-inertial) + EKF GPS fusion | Wheel odometry (Gazebo) |
| Costmap layer | `SpatioTemporalVoxelLayer` (3D) | `VoxelLayer` + `ObstacleLayer` (2D) |
| LiDAR input | `/livox/pointcloud2` (PointCloud2, via converter) | `/scan` (LaserScan) |
| `use_sim_time` | `false` | `true` |
| Dependencies | Chassis/LiDAR/RTK/FAST-LIO2 drivers | `turtlebot3_gazebo`, `gazebo_ros` |
| Package name | `carplanning_code` | `campus_nav` |

## Diagnostic Commands

```bash
# EKF real-time probe (topic frequencies, delays, TF health, GPS quality)
python3 scripts/ekf_probe.py --rate 2.0

# One-shot diagnostic snapshot
bash scripts/ekf_quick_check.sh

# Launch all diagnostics together
ros2 launch carplanning_code ekf_diagnostic.launch.py

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: -1.0, z: 0.0}, \
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

## Important Notes

- **Do NOT run Rviz2 on the Xavier NX** — CPU resources are insufficient. Run it remotely on another machine.
- **`livox_to_pc2.py` must be run separately** — the LiDAR driver outputs Livox CustomMsg, but Nav2's STVL layer expects standard PointCloud2. The converter is not auto-launched by bringup. Run: `python3 scripts/livox_to_pc2.py`
- **FAST-LIO2 outputs covariance = 0** — `lio_odom_relay.py` must be running for EKF to properly weight the odometry. Without it, EKF treats LIO as "infinitely precise" and GPS corrections are ignored.
- **`ekf.yaml` `history_length` must be a float** (e.g. `2.0`), not an integer. Using `2` causes EKF to crash with `InvalidParameterTypeException`.
- **`use_odometry_yaw: false` in navsat.yaml** — the UM982 dual antennas provide absolute heading. Setting this to `true` would use EKF's internally propagated yaw, losing the GPS heading reference.
- The URDF files (`urdf/`) and SDF model (`models/`) are simulation-only artifacts with Gazebo plugins that don't apply to the hardware branch.
- `localization.launch.py` and `nav2.launch.py` contain hardcoded paths from a previous project layout — prefer `bringup.launch.py`.
- `docs/01_PROJECT_OVERVIEW.md` contains detailed hardware specs, sensor positions, and parameter reference. `docs/03_CURRENT_ISSUES.md` tracks known bugs (costmap rotation, CPU overload, drift).
