# campus_nav（hardware 分支）

基于 RTK 的校园无图自主导航系统 —— 实物硬件版本

## 硬件配置

| 硬件     | 型号                | 话题                               |
| -------- | ------------------- | ---------------------------------- |
| 计算平台 | Jetson Xavier NX    | —                                  |
| 激光雷达 | Livox Mid-360       | `/livox/lidar` (PointCloud2)       |
| RTK 定位 | UM982 双天线        | `/gps/fix`                         |
| 底盘     | yahboomcar 四轮差速 | `/odom` `/imu/data_raw` `/cmd_vel` |

## 运行环境

- Docker 容器：Ubuntu 22.04 + ROS2 Humble
- 工作空间路径：`/ros2_ws/workspace/`

## 依赖包位置

```
/ros2_ws/workspace/
├── myslam_car/        # 底盘驱动
├── livox/             # Mid-360 驱动
├── rtk_um982/         # UM982 RTK 驱动
├── spatio_temporal/   # spatio_temporal_voxel_layer（3D costmap）
└── carplanning_code/  # 本包（导航配置+launch）
```

## 快速开始

### 1. 克隆本仓库

```bash
cd /ros2_ws/workspace
git clone https://github.com/ana52070/carplanning_code.git
cd carplanning_code
git checkout hardware
```

### 2. 编译

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/workspace/spatio_temporal/install/setup.bash
colcon build --packages-select carplanning_code
```

### 3. 启动

```bash
# source 所有工作空间
source /opt/ros/humble/setup.bash
source /ros2_ws/workspace/livox/install/setup.bash
source /ros2_ws/workspace/myslam_car/install/setup.bash
source /ros2_ws/workspace/rtk_um982/install/setup.bash
source /ros2_ws/workspace/spatio_temporal/install/setup.bash
source /ros2_ws/workspace/carplanning_code/install/setup.bash

# 一键启动
ros2 launch carplanning_code bringup.launch.py
```

### 4. 远程 Rviz（在另一台电脑上）

```bash
rviz2
# Fixed Frame: odom
# Add: PointCloud2 → /livox/lidar
# Add: Map → /local_costmap/costmap
# Add: Path → /plan
```

> ⚠️ 不要在 Xavier NX 上运行 Rviz，CPU 资源不足。

## 系统架构

```
Mid-360 (/livox/lidar) ──→ STVL (3D costmap) ──→ Nav2 costmap
                                                       ↓
UM982 (/gps/fix) ──→ navsat_transform ──→ EKF ──→ 路径规划 → /cmd_vel → 底盘
底盘 (/odom) ────────────────────────────↗
IMU (/imu/data_raw) ────────────────────↗
```

## 分支说明

| 分支         | 用途                            |
| ------------ | ------------------------------- |
| `simulation` | 仿真版本（Turtlebot3 + Gazebo） |
| `hardware`   | 实物版本（当前分支）            |

## 配置文件说明

| 文件                       | 作用                                 |
| -------------------------- | ------------------------------------ |
| `config/ekf.yaml`          | EKF 融合参数（odom + IMU + GPS）     |
| `config/navsat.yaml`       | GPS 坐标转换参数                     |
| `config/nav2_params.yaml`  | Nav2 导航参数（含 STVL 3D costmap）  |
| `launch/bringup.launch.py` | 一键启动所有节点                     |
| `launch/mid360.launch.py`  | Mid-360 雷达驱动（PointCloud2 格式） |
