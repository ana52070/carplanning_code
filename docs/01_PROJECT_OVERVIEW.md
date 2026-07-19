# 项目概述：RTK 无地图自主导航系统

> **文档版本**: 2026-06-17  
> **目标读者**: 接手此项目的技术专家  
> **工作分支**: `hardware`（真车）  
> **对照分支**: `simulation`（仿真）

---

## 1. 项目目标

基于 RTK-GPS 的**无预建地图**园区自主导航。机器人不需要事先 SLAM 建图，直接通过 RTK 定位 + 3D LiDAR 实时感知 + Nav2 导航栈实现点到点自动驾驶。

核心特性：
- **无地图导航（Mapless Navigation）**：使用 STVL (SpatioTemporalVoxelLayer) 将 3D LiDAR 点云直接投影为 2D 代价地图
- **RTK 定位融合**：通过 EKF 融合轮式里程计 + IMU + RTK-GPS 双天线航向
- **低速园区场景**：最大速度 0.26 m/s，适用于校园/园区环境

---

## 2. 硬件平台

### 2.1 计算平台

| 项目 | 型号 |
|------|------|
| 主控 | NVIDIA Jetson Xavier NX |
| 操作系统 | Ubuntu 20.04 |
| ROS 版本 | ROS2 Humble |

### 2.2 传感器与执行器

| 组件 | 型号 | 关键参数 |
|------|------|----------|
| LiDAR | Livox Mid-360 | 360° FoV, PointCloud2 格式, 10Hz |
| RTK-GPS | UM982 双天线 | 双天线测向, 串口 /dev/ttyUSB0, 115200bps |
| IMU | Yahboomcar 底盘内置 | 10Hz, 协方差全零（已知缺陷） |
| 底盘 | Yahboomcar（四轮差速） | 串口 /dev/stm32, 里程计 10Hz |

### 2.3 传感器安装位置（机器人坐标系 `base_link`）

`base_link` 为机器人几何中心（地面投影），X 向前，Y 向左，Z 向上。

| 组件 | X (前) | Y (左) | Z (高) | 说明 |
|------|--------|--------|--------|------|
| LiDAR Mid-360 | +0.14 m | 0 | +0.12 m | 水平安装 |
| GPS 天线 1 (主) | -0.14 m | +0.08 m | +0.12 m | 双天线左侧 |
| GPS 天线 2 (从) | -0.14 m | -0.08 m | +0.12 m | 双天线右侧 |
| GPS 参考点 | -0.14 m | 0 | +0.12 m | 两天线中点 |

**GPS 基线**：16 cm，左右布局（垂直于车头方向）→ `yaw_offset = π/2 (1.5708 rad)`

```
        前 (+X)
         ↑
         |   LiDAR (0.14, 0, 0.12)
         |
    ─────┼─────→ 右 (+Y)
         |
    GPS2 ●──┼──● GPS1     ← GPS 天线在后方 (-0.14)
         |
       后
```

### 2.4 其他运行的进程

Xavier NX 上同时运行：
- **ROS Bridge**：用于 Web 网页端远程控制和状态监控
- 无其他 AI/视觉进程

---

## 3. 软件架构

### 3.1 ROS2 Package 结构

```
carplanning_code/              # 纯配置包（无编译源码）
├── CMakeLists.txt             # 安装 config/ launch/ scripts/ 到 share/
├── package.xml
├── CLAUDE.md                  # AI 助手指令
├── config/
│   ├── ekf.yaml               # EKF 传感器融合参数
│   ├── ekf.yaml.orig          # 原始备份
│   ├── navsat.yaml            # GPS 坐标转换参数
│   ├── navsat.yaml.orig       # 原始备份
│   └── nav2_params.yaml       # Nav2 导航栈参数
├── launch/
│   ├── bringup.launch.py       # 主启动文件（20 个节点定时启动）
│   ├── mid360.launch.py        # Livox Mid-360 驱动启动
│   ├── ekf_diagnostic.launch.py # 诊断工具启动
│   ├── localization.launch.py  # ⚠️ 旧版，可能已废弃
│   └── nav2.launch.py          # ⚠️ 旧版，可能已废弃
├── scripts/
│   ├── covariance_relay.py     # 协方差注入节点
│   ├── ekf_probe.py            # EKF 诊断探针（Python）
│   └── ekf_quick_check.sh      # 快速诊断脚本（Bash）
└── docs/                       # 文档目录
```

### 3.2 数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                        传感器层                                       │
│                                                                       │
│  Yahboomcar 底盘          Livox Mid-360          UM982 RTK-GPS        │
│  /dev/stm32               IP: 192.168.1.140     /dev/ttyUSB0          │
│      │                          │                      │               │
│      ├─ /odom (10Hz)           ├─ /livox/lidar       ├─ /gps/fix      │
│      │  协方差=0!               │  PointCloud2        │  NavSatFix     │
│      │                          │  frame:livox_frame   │  (20Hz)        │
│      ├─ /imu/data_raw (10Hz)   │                      │               │
│      │  协方差=0!                                            │               │
│      │                          │                      │               │
└──────┼──────────────────────────┼──────────────────────┼───────────────┘
       │                          │                      │
       ▼                          │                      │
┌──────────────┐                  │                      │
│covariance_   │                  │                      │
│relay.py      │                  │                      │
│ 注入协方差    │                  │                      │
│              │                  │                      │
│ /odom/relay  │                  │                      │
│ /imu/relay   │                  │                      │
└──────┬───────┘                  │                      │
       │                          │                      │
       ▼                          │                      ▼
┌──────────────────┐             │             ┌──────────────────┐
│   ekf_node       │◄────────────┘             │ navsat_transform │
│ robot_localization│                           │   (GPS→UTM→map)  │
│                  │◄───────────────────────────│ /odometry/gps    │
│ 融合:             │   /gps/fix 输入             │ 5Hz             │
│ · odom vx,vy,vyaw│                           │ yaw_offset=π/2  │
│ · imu vyaw       │                           └──────────────────┘
│ · GPS x,y,yaw    │
│                  │
│ 输出:             │
│ /tf: map→odom    │
│      odom→base_  │
│      footprint   │
└────────┬─────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        Nav2 导航栈                                    │
│                                                                       │
│  global_costmap (map 框架)            local_costmap (odom 框架)        │
│  ┌─────────────────────┐             ┌─────────────────────┐         │
│  │ stvl_layer          │             │ stvl_layer          │         │
│  │ · topic: /livox/lidar│            │ · topic: /livox/lidar│        │
│  │ · 体素衰减: 60s      │             │ · 体素衰减: 15s      │         │
│  │ · 20m × 20m          │             │ · 3m × 3m            │         │
│  │ · rolling_window     │             │ · rolling_window     │         │
│  └─────────────────────┘             └─────────────────────┘         │
│                                                                       │
│  planner_server (Navfn)                controller_server (DWB)        │
│  ┌─────────────────────┐             ┌─────────────────────┐         │
│  │ · NavfnPlanner      │────────────►│ · DWBLocalPlanner   │         │
│  │ · 全局路径 (map)     │   路径       │ · 局部轨迹 (odom)   │         │
│  └─────────────────────┘             └──────────┬──────────┘         │
│                                                  │                    │
│  behavior_server                                 │ /cmd_vel           │
│  (spin/backup/wait)                              ▼                    │
│                                        ┌──────────────────┐          │
│                                        │ velocity_smoother │          │
│                                        │ 速度平滑          │          │
│                                        └────────┬─────────┘          │
└─────────────────────────────────────────────────┼────────────────────┘
                                                  │ /cmd_vel (平滑后)
                                                  ▼
                                            Yahboomcar 底盘
```

### 3.3 TF 坐标变换树

```
map ──(EKF)──► odom ──(底盘驱动)──► base_footprint ──► base_link
                                                          │
                                          ┌───────────────┼───────────────┐
                                          │                               │
                                    livox_frame                          gps
                                   (静态: 0.14, 0, 0.12)      (静态: -0.14, 0, 0.12)
```

- `map → odom`：EKF 输出，融合 GPS 绝对位置后修正
- `odom → base_footprint`：底盘驱动发布，纯里程计积分
- `base_link → livox_frame`：静态变换（LiDAR 安装位置）
- `base_link → gps`：静态变换（GPS 天线参考点位置）

### 3.4 节点启动时序

`bringup.launch.py` 按以下时序启动（`TimerAction` 控制延迟）：

| 延迟 | 节点 | 说明 |
|------|------|------|
| 立即 | `driver_node` | 底盘驱动（串口） |
| 立即 | GPS/LiDAR 静态 TF | `base_link→gps`, `base_link→livox_frame` |
| 2s | `livox_ros_driver2_node` | Mid-360 LiDAR 驱动 |
| 2s | `um982_ros2_driver_node` | UM982 RTK 驱动 |
| 3s | `covariance_relay` | **协方差注入** `/odom→/odom/relay`, `/imu→/imu/relay` |
| 5s | `ekf_node` | **EKF 传感器融合** (订阅 relay 后的话题) |
| 8s | `navsat_transform_node` | GPS→UTM→map 坐标转换 |
| 10s | Nav2 `navigation_launch.py` | 完整 Nav2 导航栈 |

---

## 4. 外部依赖（兄弟 Workspace）

所有依赖位于 `/ros2_ws/workspace/`：

| Workspace | 说明 | 关键输出 |
|-----------|------|----------|
| `myslam_car/` | Yahboomcar 底盘驱动 (`driver_node`) | `/odom`, `/imu/data_raw` (协方差全零) |
| `livox/` | Livox ROS2 Driver 2 (`livox_ros_driver2`) | `/livox/lidar` (PointCloud2) |
| `rtk_um982/` | UM982 RTK 驱动 (`um982_ros2_driver`) | `/gps/fix` (NavSatFix) |
| `spatio_temporal/` | STVL 代价地图插件 (`SpatioTemporalVoxelLayer`) | Nav2 costmap 插件库 |

---

## 5. 编译与运行

### 5.1 编译

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/workspace/spatio_temporal/install/setup.bash
cd /ros2_ws/workspace/carplanning_code
colcon build --packages-select carplanning_code
```

> **注意**：必须先 source `spatio_temporal` workspace，否则 STVL 插件头文件不可用。

### 5.2 运行

```bash
# 必须按顺序 source 全部依赖
source /opt/ros/humble/setup.bash
source /ros2_ws/workspace/livox/install/setup.bash
source /ros2_ws/workspace/myslam_car/install/setup.bash
source /ros2_ws/workspace/rtk_um982/install/setup.bash
source /ros2_ws/workspace/spatio_temporal/install/setup.bash
source /ros2_ws/workspace/carplanning_code/install/setup.bash

ros2 launch carplanning_code bringup.launch.py
```

### 5.3 诊断工具

```bash
# Python 探针（推荐，每2秒报告各话题频率/延迟/TF状态/GPS质量）
python3 src/scripts/ekf_probe.py --rate 2.0

# 快速诊断启动文件
ros2 launch carplanning_code ekf_diagnostic.launch.py
```

### 5.4 发送导航目标

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: -1.0, z: 0.0}, \
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

---

## 6. 关键配置文件参数速查

### 6.1 EKF (`ekf.yaml`)

| 参数 | 值 | 说明 |
|------|----|------|
| `frequency` | 5.0 Hz | Xavier NX 只能跑到 5Hz |
| `two_d_mode` | true | 2D 导航 |
| `sensor_timeout` | 0.5s | GPS 数据有效期 |
| `odom0` | `/odom/relay` | 带协方差的里程计 |
| `odom0_config` | vx, vy, vyaw | 只融合速度，不融合位置 |
| `imu0` | `/imu/relay` | 带协方差的 IMU |
| `imu0_config` | vyaw only | 只融合角速度 Z |
| `odom1` | `/odometry/gps` | GPS 绝对位置+航向 |
| `odom1_config` | x, y, yaw | 融合绝对位置和航向 |
| `history_length` | **2.0** (double) | ⚠️ 整数会导致 EKF 崩溃 |

### 6.2 Nav2 (`nav2_params.yaml`) — 为 Xavier NX 降频后

| 参数 | 值 | 原值 |
|------|----|------|
| `controller_frequency` | 10 Hz | 20 Hz |
| `expected_planner_frequency` | 5 Hz | 20 Hz |
| `bt_loop_duration` | 100 ms (10 Hz) | 10 ms (100 Hz) |
| `cycle_frequency` (behavior) | 5 Hz | 10 Hz |
| `smoothing_frequency` | 10 Hz | 20 Hz |
| `local_costmap.update_frequency` | 2 Hz | 5 Hz |
| `transform_tolerance` (全部) | **0.5s** | 0.2s |
| DWB `vx_samples` | 10 | 20 |
| DWB `vtheta_samples` | 10 | 20 |
| DWB `sim_time` | 1.5s | 1.7s |

### 6.3 协方差注入 (`covariance_relay.py`)

底盘驱动 (myslam_car_driver) 发布的 `/odom` 和 `/imu/data_raw` **协方差全为零**。EKF 将零协方差解读为"绝对精确测量"，导致融合失败。

Relay 节点注入的协方差：

| 传感器 | 物理量 | 协方差值 | 对应不确定度 (1σ) |
|--------|--------|----------|-------------------|
| Odom | vx | 0.0025 | ~0.05 m/s |
| Odom | vy | 0.0004 | ~0.02 m/s |
| Odom | vyaw | 0.0025 | ~0.05 rad/s |
| IMU | 角速度 | 0.0001 | ~0.01 rad/s |

---

## 7. `hardware` vs `simulation` 分支差异

| 方面 | `hardware` (当前) | `simulation` |
|------|-------------------|-------------|
| 代价地图层 | STVL (3D) | VoxelLayer + ObstacleLayer (2D) |
| LiDAR 输入 | `/livox/lidar` (PointCloud2) | `/scan` (LaserScan) |
| `use_sim_time` | false | true |
| 依赖 | 真实硬件驱动 | `turtlebot3_gazebo` |
| Package 名 | `carplanning_code` | `campus_nav` |

---

## 8. 注意事项

1. **不要在 Xavier NX 上运行 Rviz2** — CPU 资源不足，应在远程机器上运行
2. **`ekf.yaml` 中 `history_length` 必须是浮点数**（如 `2.0`），整数会导致 EKF 抛出 `InvalidParameterTypeException`
3. **GPS 天线位置 (x=-0.14, y=0, z=0.12) 是当前静态 TF 值**，需验证是否与实际测量一致
4. **室内测试时 GPS 无效**，导航系统无法获得绝对定位和航向参考，不应进行导航测试
5. **启动后需等待约 15 秒**让所有节点完成初始化和激活
6. **`localization.launch.py` 和 `nav2.launch.py` 包含硬编码路径**，可能是旧版遗留，优先使用 `bringup.launch.py`
