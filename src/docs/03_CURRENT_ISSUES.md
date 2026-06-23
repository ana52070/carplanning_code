# 当前问题记录：代价地图旋转与导航失败

> **记录时间**: 2026-06-17  
> **状态**: ⚠️ 未解决  
> **相关文档**: [项目概述](./01_PROJECT_OVERVIEW.md) | [合作记录](./02_COLLABORATION_LOG.md)

---

## 问题 1：代价地图旋转 / 螺旋畸变

### 1.1 症状描述

室外实车测试时：
1. 机器人静止，前方有障碍物（比如墙壁或柱子）
2. RViz2 中查看代价地图，障碍物位置可能与实际一致（初期）
3. **开始导航后**：代价地图中的障碍物发生旋转——原本在正前方的障碍物出现在右侧或左侧
4. 全局路径规划生成的方向因此错误
5. 机器人试图跟踪错误的路径，导致原地打转、螺旋移动
6. DWB 报告：`No valid trajectories out of 109! BaseObstacle/Trajectory Hits Obstacle`
7. 偶尔整个代价地图会消失几秒钟

> 用户原话："比方说现在前面有一个障碍物，方向是地图正上方。然后开始导航以后，地图会变成障碍物在正右方这样的，但是导航点还在正上方。"

### 1.2 已排除的原因

以下原因已经过代码审查和修复，**确认不是当前问题的根因**：

| 已排除 | 修复措施 | 验证结果 |
|--------|----------|----------|
| EKF 崩溃（`history_length` 类型错误） | 改为 `2.0` (double) | ✅ EKF 不再崩溃 |
| 协方差全零导致 EKF 不收敛 | `covariance_relay.py` 注入协方差 | ✅ `map→odom` TF 正常发布 |
| 缺失 `base_link→gps` 静态 TF | 添加静态变换 | ✅ navsat 不再报该错误 |
| `transform_tolerance` 太紧 (0.2s) 导致点云被丢弃 | 改为 0.5s | ✅ "Message Filter dropping" 消失 |
| Xavier NX CPU 过载 (全部节点频率过高) | 全局降频 | ✅ EKF 稳态不掉频, BT 超时减少 |
| GPS 航向被忽略 (`use_odometry_yaw: true` + `odom1_config` 无 yaw) | 改为 `false` + 加 yaw | ❓ 修复后问题依旧 |

### 1.3 当前推测

修复了 GPS 航向融合后问题依然存在，可能的原因：

#### 假说 A：GPS 航向数据本身有问题

`use_odometry_yaw: false` 时，navsat_transform 从 GPS 获取航向。但需要确认：
1. UM982 驱动是否实际向 navsat_transform 提供了航向数据？
2. 航向数据是否正确？（`yaw_offset: 1.5708` 是否与实际天线安装匹配）
3. 航向的协方差是否合理（过大则 EKF 权重太低，过小则可能被异常值污染）

**验证方法**：
```bash
# 查看 /odometry/gps 的 orientation 是否正确
ros2 topic echo /odometry/gps --field pose.pose.orientation

# 查看 navsat_transform 的日志, 确认航向来源
ros2 run rqt_reconfigure rqt_reconfigure  # 动态调 yaw_offset 验证
```

#### 假说 B：EKF 初始化时的 yaw 跳变

EKF 启动时 yaw 初始化为 0（从里程计初始值）。当第一个 GPS 测量到达时，如果 GPS 航向与当前 EKF yaw 差距大（例如 90°），EKF 会有一个**瞬间 yaw 跳变**。这个跳变导致：
- `map→odom` TF 突然旋转
- 已构建的代价地图瞬间失效（障碍物位置错位）
- 全局规划器基于旧 TF 生成的路径失效

**验证方法**：
```bash
# 监控 /tf 中 map→odom 的 orientation 变化
ros2 topic echo /tf | grep -A20 "map"

# 或者用 tf2_echo 持续监控
ros2 run tf2_ros tf2_echo map odom
```

#### 假说 C：`odom→base_link` TF 的 yaw 精度问题

EKF 发布 `map→odom`，底盘驱动发布 `odom→base_link`。最终 `map→base_link` = `map→odom` × `odom→base_link`。

如果底盘驱动的 `odom→base_link` 本身的 yaw 就有漂移（陀螺仪零偏未校准），那么即使 `map→odom` 的 yaw 被 GPS 纠正，整体 yaw 也会随着积分漂移。

**验证方法**：
```bash
# 静止状态下监控 odom→base_link 的累积旋转
ros2 run tf2_ros tf2_echo odom base_link
# 观察 yaw 是否随时间缓慢增加
```

#### 假说 D：`base_footprint` vs `base_link` 坐标系问题

TF 树中有 `odom→base_footprint→base_link`。Nav2 的 `robot_base_frame` 设置为 `base_link`，但成本地图可能使用 `base_footprint`。两者之间的变换如果缺失或错误，会导致机器人位置投影偏差。

**当前 TF 情况**：
- `odom→base_footprint`：由底盘驱动发布
- `base_footprint→base_link`：？？（需要确认是否存在）

```bash
# 检查完整 TF 链
ros2 run tf2_tools view_frames
# 确认 base_footprint→base_link 变换存在
```

---

## 问题 2：Xavier NX CPU 过载

### 2.1 症状

- EKF 初始化阶段掉频严重（`Took 0.94s` 单次更新）
- Nav2 配置/激活阶段 CPU 竞争激烈
- 所有节点同时竞争 → 级联延迟

### 2.2 当前措施

已在 `nav2_params.yaml` 中全局降低频率（详见 [合作记录](./02_COLLABORATION_LOG.md#阶段-8xavier-nx-cpu-过载调优)）。

### 2.3 进一步优化建议

1. **使用 ROS2 的 CPU 亲和性 (`taskset`)**：将关键节点（EKF、controller_server）绑定到不同 CPU 核
2. **关闭不必要的 Nav2 插件**：移除未使用的 behavior 插件（如 `assisted_teleop`）
3. **减少 STVL 体素分辨率**：`voxel_size: 0.05` → `0.1` 可减少 75% 体素数量
4. **LiDAR 点云下采样**：在 Livox 驱动中添加体素滤波，减少输入点云密度
5. **使用 `cyclonedds` RMW**：替换默认的 `fastrtps`，减少 DDS 通信开销

```bash
# 临时启用 cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch carplanning_code bringup.launch.py
```

---

## 问题 3：静止漂移

### 3.1 症状

机器人静止不动时，RViz2 中显示的位置缓慢漂移（缓慢移动或旋转）。

### 3.2 分析

GPS 质量为 DGPS（不是 RTK 固定解）时，GPS 定位精度约 ±1-3m。EKF 在融合 GPS 时会把位置估计往 GPS 读数拉。如果 GPS 读数本身有慢漂移（DGPS 的典型现象），机器人位置也会跟着漂移。

### 3.3 建议

1. 等待 GPS 收敛到 RTK 固定解（质量标记从 DGPS 变为 FIX）
2. 在 `ekf.yaml` 中增加 GPS 位置融合的测量噪声（需要修改 `/odometry/gps` 消息中的协方差）
3. 静止时关闭 GPS 融合（需要应用层逻辑判断）

> **注意**：这个问题在 GPS 达到 RTK 固定后应该自然消失。

---

## 问题 4：导航开始后原地打转

### 4.1 症状

1. 发送导航目标后
2. 机器人先执行 `spin` 行为（原地转 1.57 rad = 90°）
3. 之后反复执行 `backup` / `spin` / `wait` 恢复行为
4. 偶尔 `controller_server` 短时间成功生成控制指令
5. 但随即报告 `No valid trajectories` 并再次触发恢复
6. 整体表现：机器人原地反复旋转、小幅前后移动

### 4.2 分析

这是问题 1 的**直接后果**：

```
代价地图方向错误 → 全局路径指向错误方向
  → DWB 尝试生成到目标点的局部轨迹
    → 代价地图显示所有方向都有障碍物 (方向错了)
      → 419→109 条轨迹全部被 BaseObstacle 拒绝
        → 触发 recovery (spin/backup/wait)
          → recovery 改变机器人朝向
            → 但代价地图方向仍然错误
              → 再次失败 → 无限循环
```

### 4.3 关键日志证据

从最新实车测试日志（2026-06-17 09:04 & 09:15）：

```
# 导航开始
[bt_navigator]: Begin navigating from current location (0.20, -0.65) to (5.15, -0.88)

# DWB 控制失败 - 所有轨迹被障碍物拒绝
[DWBLocalPlanner]: No valid trajectories out of 419!
[DWBLocalPlanner]: 1.00: BaseObstacle/Trajectory Hits Obstacle.

# 全局路径规划也失败
[planner_server]: GridBased: failed to create plan with tolerance 0.50.
[planner_server]: Planning algorithm GridBased failed to generate a valid path

# 进入恢复模式循环
[behavior_server]: Running spin        → Turning 1.57 for spin behavior
[behavior_server]: Running backup
[behavior_server]: Running wait

# 偶尔成功生成控制
[controller_server]: Passing new path to controller.
# 但随即又失败
[DWBLocalPlanner]: No valid trajectories out of 109!
```

### 4.4 第二次测试（频率降低后）

降频后 DWB 轨迹数从 419 降至 109（采样减半效果），但问题本质未变：
```
[DWBLocalPlanner]: No valid trajectories out of 109!
[DWBLocalPlanner]: 0.54: BaseObstacle/Trajectory Hits Obstacle.
[DWBLocalPlanner]: 0.46: Oscillation/Trajectory is oscillating.
```

新出现 `Oscillation` 拒绝（46% 轨迹因振荡被拒），说明机器人在往复打转。

---

## 综合诊断步骤（建议后续专家按顺序排查）

### Step 1：验证 GPS 航向

```bash
# 在室外 (GPS 有信号) 启动后, 检查 /odometry/gps 的 orientation
ros2 topic echo /odometry/gps --field pose.pose.orientation --once

# 同时检查 /gps/fix 中是否有航向信息
ros2 topic echo /gps/fix --once
```

### Step 2：验证 TF 链 yaw 正确性

```bash
# 将机器人与真实环境对齐后, 监控 map→base_link 的 yaw
ros2 run tf2_ros tf2_echo map base_link

# 让机器人执行原地旋转, 观察 RViz 中旋转方向与真实世界是否一致
```

### Step 3：验证 yaw_offset

当前 `yaw_offset: 1.5707963` (π/2)。如果 GPS 基线不是严格左右布局，需要调整。

**调试方法**：让机器人直线前进，在 RViz 中观察轨迹方向：
- 如果轨迹方向与真实方向偏差 90°，说明 `yaw_offset` 错误
- 如果偏差随机变化，说明 GPS 航向数据本身有问题

### Step 4：检查 `base_footprint→base_link` TF

```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf, 检查 base_footprint→base_link 是否存在
```

### Step 5：降低系统负载做对照实验

```bash
# 临时关闭 web bridge, 释放 CPU
# 或单独测试 EKF+navsat_transform (不启动 Nav2)
# 验证定位本身是否正常
```

### Step 6：排查 navsat_transform 航向来源

查看 navsat_transform 源码或日志，确认 `use_odometry_yaw: false` 时：
1. 它从哪个话题获取航向？（/gps/fix? 单独的 /heading 话题？）
2. UM982 驱动是否发布了该话题？
3. 航向值的坐标系与 robot_localization 期望的坐标系是否一致？

---

## 环境信息

| 项目 | 值 |
|------|-----|
| ROS2 版本 | Humble |
| 平台 | Jetson Xavier NX |
| 分支 | `hardware` |
| 同时运行的进程 | ros_bridge (Web 控制) |
| GPS 质量 | DGPS（测试时），未达 RTK 固定 |
| 测试场地 | 室外道路，无高大建筑 |
