# 无图导航系统 — 代价地图漂移问题诊断与修复报告

> **日期**: 2026-06-23  
> **分支**: `hardware`  
> **平台**: NVIDIA Jetson Xavier NX + Livox Mid-360 + UM982 RTK + Yahboomcar 底盘  
> **定位方案**: FAST-LIO2 (LiDAR-惯性里程计) + Nav2 (STVL 3D 代价地图)  

---

## 1. 问题现象

室外/室内导航测试中：

1. **代价地图与点云不对齐** — RViz 中实时点云和代价地图的障碍物位置发生偏转，原本正前方的障碍物显示在侧面
2. **导航开始后机器人原地打转** — DWB 报告 `No valid trajectories out of 109!`，`cmd_vel` 角速度命令振荡在 ±1.0 rad/s（最大限制值）
3. **漂移随时间加剧** — 导航约 50 秒后系统彻底崩溃，LIO 位姿 yaw 在 ±180° 间剧烈跳变
4. **无论有无 EKF/GPS 融合，问题始终存在** — 即使简化为纯 FAST-LIO2 定位（去除 EKF），问题依旧

---

## 2. 诊断过程

### 2.1 第一阶段：排除 EKF 和 TF 冲突

**初始架构**：
```
FAST-LIO2 → EKF ← GPS → map→odom TF → Nav2
```

**尝试**：移除 EKF 和 navsat_transform，改用静态 `map→odom` identity TF，Nav2 在 `odom` 帧内运行。**问题依旧**，说明根因不在 EKF/GPS 回路。

**TF 树验证**：
```
map → odom (静态 identity)
odom → base_link (10.266Hz)
base_link → livox_frame (静态)
base_link → gps (静态)
```
TF 树没有冲突，单一 broadcaster 发布每段变换。

### 2.2 第二阶段：发现 FAST-LIO2 TF 缺失

查看 FAST-LIO2 源码 `lio_node.cpp` 第 260 行：

```cpp
//broadCastTF(m_tf_broadcaster, m_node_config.world_frame,
//            m_node_config.body_frame, m_package.cloud_end_time);
```

**FAST-LIO2 的 TF 广播被注释掉了。** 系统中的 `odom→base_link` TF 实际来自底盘驱动的**轮式里程计**（`carplanning_driver.py` 第 281-299 行），这是编码器积分的低精度数据，存在严重漂移。

> **发现**：FAST-LIO2 虽然计算了高质量的 LiDAR-惯性里程计（发布在 `/fastlio2/lio_odom` 话题），但从未发布 TF。整个导航栈使用的 `odom→base_link` TF 来自底盘驱动的轮式里程计。

### 2.3 第三阶段：修复 TF 后仍漂移

启用 FAST-LIO2 的 TF 广播后，写了一个漂移监控脚本 `drift_monitor.py`，每 2 秒采样以下指标：

| 指标 | 意义 |
|------|------|
| `lio_x/y/yaw` | LIO 位姿 |
| `drift_x/y/yaw` | 相对初始位姿的累积漂移 |
| `yaw_rate_window` | 近期 yaw 变化率 (°/s) |
| `tf_age_ms` | TF 时间戳年龄（距当前时间） |
| `cmd_vx/vz` | DWB 输出的速度指令 |
| `costmap_obs_ratio` | 代价地图障碍物占比 |

**第一次完整测试数据（drift_monitor_20260623_114737.csv）关键发现**：

```
时间   | yaw      | TF年龄  | yaw变化率  | cmd_vz
-------|----------|---------|------------|-------
10-34s | -59~-97° | 71ms    | ±10°/s     | ±0.3   ← 第一阶段：勉强正常
34-52s | -80.6°   | 71ms    | 0°/s       | 0      ← 静止：完美稳定
52s    | -80.7°   | 174ms   | -          | 0.26   ← 第二阶段导航开始
54s    | -110.3°  | 575ms   | -          | 0.56   ← TF开始延迟
56s    | -79.2°   | 1078ms  | -          | -0.11  ← TF延迟超1秒!
58s    | -16.4°   | 1271ms  | -          | -1.0   ← TF延迟1.3秒
60s    | -147.5°  | 1073ms  | -          | 1.0    ← cmd_vz达到极限
62s    | -178.4°  | 1078ms  | -53.8°/s   | 1.0    ← yaw崩溃
68s    | -157.9°  | 1172ms  | -25.5°/s   | 0      ← Z轴跳变到+0.41m
70s    | +145.9°  | 1173ms  | +43.3°/s   | 0      ← LIO彻底丢失跟踪
```

**铁证**：TF 年龄从正常的 71ms 飙升到 1271ms（18 倍），说明 FAST-LIO2 的 `timerCB` 计算跟不上 LiDAR 帧率。

### 2.4 根因确认

**直接原因**：FAST-LIO2 的 IESKF 迭代 + ikd-Tree 地图维护的单次耗时超过 LiDAR 帧间隔（100ms），导致 `timerCB` 堆积，TF 发布时间戳滞后 1 秒以上。代价地图用 1 秒前的位姿投影当前点云，障碍物位置严重错位，DWB 做出错误避障决策，机器人开始猛烈打转，产生更多点云数据，形成恶性循环。

**根本原因**：Xavier NX 的 ARM CPU 算力不足以支撑 FAST-LIO2 的默认参数配置（300m³ 地图、60m 检测范围、5 次 IESKF 迭代、密集点云降采样）。

**证据链**：
1. 静止时 yaw 完美稳定（-80.6° ± 0.1°），说明算法本身正确
2. TF 年龄从 71ms → 1271ms 的飙升与导航开始精确同步
3. LIO z 轴跳变到 +0.41m 是 IESKF 失去跟踪的典型症状
4. `cmd_vz=±1.0` 是 DWB 角速度硬限制，说明控制器已"绝望"

---

## 3. 解决方案

### 3.1 修复 FAST-LIO2 TF 广播缺失

**文件**: `fastlio2_ws/src/FASTLIO2_ROS2/fastlio2/src/lio_node.cpp:260`

```cpp
// 修改前（被注释）:
//broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

// 修改后（已启用）:
broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);
```

### 3.2 禁用底盘驱动的轮式里程计 TF

**文件**: `myslam_car/.../carplanning_driver.py` 第 281-300 行

底盘驱动的 `odom→base_link` TF 广播已注释。底盘驱动继续运行其他功能（`/cmd_vel` 接收、`/odom` 话题备用、`/imu/data_raw` 发布），但不再发布 TF。同时保留了四元数计算 `q = tf_transformations.quaternion_from_euler(...)`，因为 `/odom` 话题消息仍需要它。

### 3.3 移除 FAST-LIO2 的 RViz2 节点

**文件**: `fastlio2_ws/src/FASTLIO2_ROS2/fastlio2/launch/lio_launch.py`

原始 launch 文件会同时启动 RViz2，对 Xavier NX 的 CPU 是灾难。已移除 RViz2 节点。

### 3.4 FAST-LIO2 参数优化（关键）

**文件**: `fastlio2_ws/src/FASTLIO2_ROS2/fastlio2/config/lio.yaml`

| 参数 | 原值 | 新值 | 效果 |
|------|------|------|------|
| `lidar_filter_num` | 6 | **10** | 输入点数减少 ~40% |
| `lidar_max_range` | 30m | **20m** | 裁掉远处噪声 |
| `scan_resolution` | 0.15m | **0.3m** | 体素降采样更粗 |
| `map_resolution` | 0.3m | **0.5m** | ikd-Tree 节点数大幅减少 |
| `cube_len` | 300m | **100m** | 地图边界缩小 3× |
| `det_range` | 60m | **30m** | 检测范围减半 |
| `near_search_num` | 5 | **3** | 最近邻搜索量 -40% |
| `ieskf_max_iter` | 5 | **3** | 迭代次数 -40% |
| `print_time_cost` | false | **true** | 打印每次迭代耗时（诊断用） |

### 3.5 降低 FAST-LIO2 timer 频率

**文件**: `fastlio2_ws/src/FASTLIO2_ROS2/fastlio2/src/lio_node.cpp:67`

```cpp
// 修改前:
m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));

// 修改后:
m_timer = this->create_wall_timer(30ms, std::bind(&LIONode::timerCB, this));
```

### 3.6 Nav2 参数优化

**文件**: `src/config/nav2_params.yaml`

| 参数 | 原值 | 新值 | 效果 |
|------|------|------|------|
| STVL `voxel_size` (局部+全局) | 0.05m | **0.1m** | 体素数量减少 ~87.5% |
| STVL `voxel_decay` (全局) | 60s | **30s** | 旧体素更快过期 |
| `behavior_plugins` | 5个 | **3个** | 移除 `drive_on_heading`、`assisted_teleop` |

### 3.7 屏蔽底盘驱动崩溃 bug

**文件**: `myslam_car/.../carplanning_driver.py`

由于 `q` 变量在 TF 代码块中被注释但被 odometry 发布代码引用，导致 `NameError: name 'q' is not defined` 崩溃。已将四元数计算移到 TF 注释块之前独立保留。

---

## 4. 诊断工具

开发了 `drift_monitor.py` 漂移监控脚本，位于 `src/scripts/drift_monitor.py`。

**使用方法**：
```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/fastlio2_ws/install/setup.bash
python3 src/scripts/drift_monitor.py --log /tmp/drift.csv
```

**监控指标**：LIO 位姿、累积漂移（x/y/yaw）、yaw 变化率（°/s）、TF 时间戳年龄、代价地图障碍物比例、全局路径状态、cmd_vel 输出。

---

## 5. 验证结果

### 优化前 vs 优化后对比

| 指标 | 优化前 | 优化后 |
|------|--------|--------|
| TF 最大年龄 | **1271ms** | **135ms** |
| TF 稳定年龄 | 70ms→1078ms 持续飙升 | 128-132ms 恒定 |
| yaw 振荡 | ±180° 来回跳变 | 平缓过渡 |
| z 轴漂移 | 跳变到 +0.41m（LIO 失跟） | 最大 -0.14m |
| 导航段数 | 第 1 段勉强，第 2 段崩溃 | **4 段全部成功** |
| cmd_vz 振荡 | ±1.0 (极限值) | ±0.3~0.8 |

### 优化后典型导航轨迹

```
起点 (0, 0)
  → 第1段 3.5m 直线，yaw -58° 稳定到达 (1.98, -3.48)
  → 第2段 继续前进到 (4.92, -3.21)
  → 第3段 返航 4.5m 到 (0, -0.78)
  → 第4段 新方向到 (-1.60, 2.52)
全程 TF 年龄 128-135ms，0 次超时
```

---

## 6. 修改文件清单

| 文件 | 改动类型 | 说明 |
|------|----------|------|
| `fastlio2/.../lio_node.cpp:260` | 取消注释 | 启用 TF 广播 |
| `fastlio2/.../lio_node.cpp:67` | 参数修改 | timer 20ms→30ms |
| `fastlio2/.../lio_launch.py` | 删除节点 | 移除 RViz2 |
| `fastlio2/.../lio.yaml` | 参数优化 | 8 项参数降低计算量 |
| `myslam_car/.../carplanning_driver.py` | 注释+修复 | 禁用轮式里程计 TF，修复 q 变量 |
| `src/config/nav2_params.yaml` | 参数优化 | STVL voxel_size、voxel_decay、behavior_plugins |
| `src/scripts/drift_monitor.py` | 新增文件 | 漂移诊断监控脚本 |

---

## 7. 后续建议

### 7.1 短期（本周）

1. **室内多轮导航测试** — 验证系统在连续多段导航中的稳定性
2. **监控 `Time cost`** — FAST-LIO2 终端会打印每次 IESKF 迭代耗时，必须 < 100ms
3. **测试完成后关闭 `print_time_cost`** — 减少日志 I/O 开销

### 7.2 中期

1. **GPS 目标点转换节点** — 实现经纬度→odom 帧 XY 坐标的一次性转换，GPS 不参与持续定位
2. **室外 RTK 环境测试** — 在 GPS RTK 固定解可用时测试完整无图导航流程
3. **启用 CycloneDDS** — `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，预计额外节省 10-20% CPU

### 7.3 长期

1. **考虑 odom 帧漂移累积** — FAST-LIO2 在长距离（>50m）或长时间（>5min）运行后会累积漂移。如需要，可定期用 GPS 做一次性位置修正（不参与持续融合）
2. **FAST-LIO2 外参标定** — 当前 `t_il: [-0.011, -0.02329, 0.04412]` 可能是默认值，建议用标定工具获取真实 IMU-LiDAR 外参
3. **Xavier NX 替代方案评估** — 如果后续需要加入视觉或 AI 模块，建议评估 Jetson Orin NX 或 Orin Nano

---

## 附录 A：关键诊断命令

```bash
# TF 来源确认
ros2 run tf2_ros tf2_echo odom base_link

# LIO 话题检查
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /livox/imu

# 点云数据确认
ros2 topic hz /livox/pointcloud2

# 漂移监控
python3 src/scripts/drift_monitor.py --log /tmp/drift.csv

# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, \
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

## 附录 B：启动流程

```bash
# 0. 环境变量
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # 可选，降低 DDS 开销

# 1. 启动主系统
source /opt/ros/humble/setup.bash
source /home/ubuntu/fastlio2_ws/install/setup.bash
source /home/ubuntu/ros2_ws/workspace/livox/install/setup.bash
source /home/ubuntu/ros2_ws/workspace/myslam_car/install/setup.bash
source /home/ubuntu/ros2_ws/workspace/carplanning_code/install/setup.bash
ros2 launch carplanning_code bringup.launch.py

# 2. 点云格式转换（必须！）
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/workspace/livox/install/setup.bash
python3 /home/ubuntu/ros2_ws/workspace/carplanning_code/src/scripts/livox_to_pc2.py

# 3. 漂移监控（可选）
source /opt/ros/humble/setup.bash
source /home/ubuntu/fastlio2_ws/install/setup.bash
python3 /home/ubuntu/ros2_ws/workspace/carplanning_code/src/scripts/drift_monitor.py
```

## 附录 C：当前 TF 树和定位架构

```
静态 TF: map → odom (identity, 10000Hz)

FAST-LIO2: odom → base_link (10Hz, LiDAR-惯性里程计, 高精度)
  ├── base_link → livox_frame (静态: x=0.14, z=0.26)
  ├── base_link → gps (静态: x=-0.14, z=0.12)
  └── 底盘驱动不再发布 odom→base_link TF

Nav2 在 odom 帧内运行 (global_frame: odom, robot_base_frame: base_link)
GPS 不参与持续定位，仅用于目标点经纬度→XY 一次性转换
```
