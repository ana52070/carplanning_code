# carplanning_code

基于 RTK + FAST-LIO2 的校园无图自主导航系统 —— 硬件分支

## 硬件配置

| 硬件 | 型号 | 话题 |
|------|------|------|
| 计算平台 | Jetson Xavier NX | — |
| 激光雷达 | Livox Mid-360 | `/livox/lidar` (CustomMsg) |
| RTK 定位 | UM982 双天线 | `/gps/fix` |
| 底盘 | Yahboomcar 四轮差速 | `/odom`, `/imu/data_raw`, `/cmd_vel` |

## 运行环境

- ROS2 Humble (Ubuntu 22.04)
- 主工作空间：`/ros2_ws/workspace/carplannning_workspace`

## 依赖包

本包作为 [carplannning_workspace](https://github.com/ana52070/carplannning_workspace) 的子模块使用，依赖以下同级工作空间：

```
carplannning_workspace/
├── myslam_car/              # 亚博底盘驱动
├── livox/                   # Livox Mid-360 驱动
├── rtk_um982/               # UM982 RTK GPS 驱动
├── spatio_temporal/         # STVL 3D 代价地图插件
├── fastlio2/                # FAST-LIO2 激光惯性里程计
└── carplanning_code/        # 本包（导航启动+配置）
    └── src/ → 子模块
```

## 快速开始

### 1. 编译

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/workspace/spatio_temporal/install/setup.bash
colcon build --packages-select carplanning_code
```

### 2. 启动

```bash
# source 所有工作空间
source /opt/ros/humble/setup.bash
source /ros2_ws/workspace/livox/install/setup.bash
source /ros2_ws/workspace/myslam_car/install/setup.bash
source /ros2_ws/workspace/rtk_um982/install/setup.bash
source /ros2_ws/workspace/spatio_temporal/install/setup.bash
source /ros2_ws/workspace/fastlio2/install/setup.bash
source /ros2_ws/workspace/carplanning_code/install/setup.bash

# 一键 bringup
ros2 launch carplanning_code bringup.launch.py

# 另开终端：启动 LiDAR 格式转换（STVL 代价地图必须）
python3 scripts/livox_to_pc2.py
```

### 3. 远程 RViz（在另一台电脑上）

```bash
rviz2
# Fixed Frame: odom
# Add: PointCloud2 → /livox/pointcloud2
# Add: Map → /local_costmap/costmap
# Add: Path → /plan
```

> ⚠️ 不要在 Xavier NX 上运行 RViz，CPU 资源不足。

## 系统架构

### 数据流

```
Livox Mid-360 (/livox/lidar, CustomMsg)
    ├──► FAST-LIO2 (紧耦合 IMU) ──► /fastlio2/lio_odom (cov=0)
    │         └──► lio_odom_relay (注入协方差) ──► /lio_odom/relay
    │                   └──► EKF ← /odometry/gps ← navsat_transform ← /gps/fix
    │                           └──► map → odom TF
    │
    └──► livox_to_pc2 (格式转换) ──► /livox/pointcloud2
              └──► STVL 3D 代价地图 ──► Nav2 ──► /cmd_vel ──► 底盘
```

双里程计管线：
- **FAST-LIO2**（主力）：激光惯性里程计，高频 (~33Hz)，漂移缓慢累积
- **底盘轮式里程计** (`/odom`)：仅用于 `odom→base_footprint` TF，不参与 EKF 融合

GPS 通过 EKF 提供绝对位置/航向修正，约束 FAST-LIO2 漂移。

### TF 树

```
map ──(EKF)──► odom ──(底盘)──► base_footprint ──► base_link
                                                       │
                            ┌──────────────────────────┤
                            │                          │
                      livox_frame                    gps
                   (LiDAR, z=0.26)             (天线, z=0.12)
```

### bringup 节点启动顺序（含延时）

| 延时 | 节点 | 作用 |
|------|------|------|
| 立即 | `myslam_car_driver` | 底盘驱动：`/odom`, `/imu/data_raw`, `odom→base_footprint` TF |
| 立即 | 静态 TF | `base_link→gps`, `base_link→livox_frame` |
| 2s | Livox Mid-360 | 激光雷达：`/livox/lidar` (CustomMsg) |
| 2s | UM982 驱动 | GPS：`/gps/fix` |
| 4s | FAST-LIO2 | 激光惯性里程计：`/fastlio2/lio_odom` |
| 5s | `lio_odom_relay` | 注入协方差 → `/lio_odom/relay` |
| 7s | `ekf_node` | 传感器融合：LIO + GPS → `map→odom` TF |
| 10s | `navsat_transform` | GPS→UTM→map 坐标转换 |
| 12s | Nav2 | 完整导航框架 |

## 分支说明

| 分支 | 用途 |
|------|------|
| `simulation` | 仿真版本（Turtlebot3 + Gazebo） |
| `hardware` | 实物版本（当前分支，默认） |

## 配置文件

| 文件 | 作用 |
|------|------|
| `config/ekf.yaml` | EKF 融合参数（LIO 微分模式 + GPS 绝对观测，2D） |
| `config/navsat.yaml` | GPS 坐标转换（`use_odometry_yaw: false`，使用 UM982 双天线航向） |
| `config/nav2_params.yaml` | Nav2 导航参数（STVL 3D 代价地图，Navfn + DWB） |
| `launch/bringup.launch.py` | 一键 bringup |
| `launch/mid360.launch.py` | Mid-360 雷达配置（CustomMsg 格式） |
| `launch/ekf_diagnostic.launch.py` | EKF 诊断工具集 |

## 脚本

| 脚本 | 用途 |
|------|------|
| `scripts/lio_odom_relay.py` | 给 FAST-LIO2 里程计注入协方差（原输出 cov=0） |
| `scripts/livox_to_pc2.py` | Livox CustomMsg → PointCloud2 格式转换（**需手动启动**） |
| `scripts/ekf_probe.py` | EKF 实时诊断探针（频率、延迟、TF 健康） |
| `scripts/ekf_quick_check.sh` | 一次性诊断快照 |
| `scripts/drift_monitor.py` | GPS/里程计漂移监控 |
| `scripts/wait_for_tf.py` | TF 就绪等待工具 |
| `scripts/covariance_relay.py` | 旧版协方差注入（EKF v2，已弃用） |

## 注意事项

- **`livox_to_pc2.py` 不会自动启动** — bringup 不包含它，需手动运行
- **FAST-LIO2 输出 covariance = 0** — `lio_odom_relay.py` 必须运行，否则 EKF 将 LIO 视为"无限精确"而忽略 GPS 修正
- **`ekf.yaml` 中 `history_length` 必须为浮点数**（如 `2.0`），整数会导致 EKF 崩溃
- **`use_odometry_yaw: false`** — UM982 双天线提供绝对航向，设为 true 会丢失 GPS 航向参考
- **构建顺序** — 本包依赖 `spatio_temporal_voxel_layer` 头文件，需先编译 spatio_temporal

详细文档：`docs/01_PROJECT_OVERVIEW.md`
已知问题：`docs/03_CURRENT_ISSUES.md`
