# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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
source /ros2_ws/workspace/carplanning_code/install/setup.bash
ros2 launch carplanning_code bringup.launch.py
```

There are no tests in this package.

## Architecture

### Data flow (hardware branch)

```
Livox Mid-360 (/livox/lidar, PointCloud2)
       ↓
STVL 3D costmap layer (spatio_temporal_voxel_layer)
       ↓
Nav2 costmap → Path planning (Navfn + DWB) → /cmd_vel → Chassis

UM982 (/gps/fix) → navsat_transform → /odometry/gps → EKF ← /odom (chassis)
                                                   ↑       ← /imu/data_raw (chassis)
                                              sensor fusion (ekf_node)
                                                   ↓
                                          map → odom → base_footprint TF
```

### TF tree

- `map` → `odom` → `base_footprint` (published by EKF)
- `base_link` → `livox_frame` (static: x=0.14, z=0.12)

### bringup.launch.py node startup order (with delays)

1. `myslam_car_driver` — chassis driver (immediate)
2. Livox Mid-360 (`mid360.launch.py`) — LiDAR driver, PointCloud2 format (2s)
3. `um982_ros2_driver` — RTK GPS driver (2s)
4. `ekf_node` (robot_localization) — sensor fusion: odom + IMU + GPS (5s)
5. `navsat_transform_node` — GPS-to-map coordinate transform (8s)
6. Nav2 `navigation_launch.py` — full navigation stack with custom params (10s)
7. `static_transform_publisher` — base_link→livox_frame (immediate)

### Key config files

| File | Purpose |
|------|---------|
| `config/ekf.yaml` | EKF fusion params (2D mode, 20Hz, fuses `/odom` + `/imu/data_raw` + `/odometry/gps`) |
| `config/navsat.yaml` | GPS coordinate transform (yaw offset π/2, zero altitude) |
| `config/nav2_params.yaml` | Nav2 stack: STVL 3D costmap observing `/livox/lidar`, Navfn planner, DWB controller (max 0.26 m/s) |
| `config/nav2_params.yaml.bak` | Backup with standard 2D costmap layers + `/scan` for simulation reference |

### External dependencies (sibling workspaces)

All at `/ros2_ws/workspace/`:
- `myslam_car/` — Yahboomcar chassis driver
- `livox/` — Livox Mid-360 LiDAR driver (`livox_ros_driver2`)
- `rtk_um982/` — UM982 dual-antenna RTK GPS driver (`um982_ros2_driver`)
- `spatio_temporal/` — Custom STVL 3D costmap plugin for Nav2

### Key differences: hardware vs simulation branch

| Aspect | `hardware` (current) | `simulation` |
|--------|---------------------|-------------|
| Costmap layer | `SpatioTemporalVoxelLayer` (3D) | `VoxelLayer` + `ObstacleLayer` (2D) |
| LiDAR input | `/livox/lidar` (PointCloud2) | `/scan` (LaserScan) |
| `use_sim_time` | `false` | `true` |
| Dependencies | Chassis/LiDAR/RTK drivers | `turtlebot3_gazebo`, `gazebo_ros` |
| Package name | `carplanning_code` | `campus_nav` |

## Important Notes

- **Do NOT run Rviz2 on the Xavier NX** — CPU resources are insufficient. Run it remotely on another machine.
- The URDF files (`urdf/turtlebot3_waffle.urdf`) and SDF model (`models/`) are simulation-only artifacts and include Gazebo plugins that don't apply to the hardware branch.
- `localization.launch.py` and `nav2.launch.py` contain hardcoded paths from a previous project layout and are likely legacy — prefer `bringup.launch.py`.
- The git working tree has unstaged deletions at the repo root from a restructuring that moved everything into `src/`.
