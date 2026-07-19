# 合作调试记录：EKF 定位与 Nav2 导航问题修复

> **记录时间**: 2026-06-17  
> **合作者**: 用户（硬件/测试） + Claude Code（诊断/修复）  
> **分支**: `hardware`  
> **原始状态**: 系统可以启动，但存在多种 EKF 和导航问题

---

## 阶段 1：系统初始化与文档

### 1.1 创建 CLAUDE.md

为代码库创建了 AI 助手指令文件，包含：
- 编译命令、工作空间路径
- 架构概述、数据流
- 配置文件索引
- `hardware` vs `simulation` 分支差异
- 常见陷阱（Xavier NX 性能限制等）

### 1.2 创建架构图

在 `docs/` 下创建了：
- `architecture.svg`：双栏 SVG 架构图（硬件 vs 仿真对比，中文标注）
- `architecture.md`：中文架构说明文档（5 层架构、TF 树、参数对比表）

---

## 阶段 2：EKF 崩溃修复

### 2.1 症状

启动后 EKF 立即崩溃：

```
[ekf_node]: parameter 'history_length' has invalid type: Wrong parameter type,
parameter {history_length} is of type {double}, setting it to {integer} is not allowed.
```

### 2.2 根因

`robot_localization` (ROS2 Humble) 对参数类型检查严格。YAML 中 `history_length: 2` 被解析为整数，而 EKF 期望 double 类型。

同时，以下参数在 Humble 版本中不存在或行为不同：
- `odom0_config_with_covariance` / `odom1_config_with_covariance`
- `odom0_relative` / `imu0_relative` / `odom1_relative`
- `reset_on_time_jump`

### 2.3 修复

**文件**: `config/ekf.yaml`

```yaml
# 关键修复点
history_length: 2.0          # ⚠️ 必须是浮点数! 整数 (2) 会导致 EKF 崩溃!
smooth_lagged_data: true
predict_to_current_time: true

# 移除了以下不兼容参数：
# - odom0_config_with_covariance
# - odom1_config_with_covariance
# - odom0_relative / imu0_relative / odom1_relative
# - reset_on_time_jump
```

备份原始配置为 `config/ekf.yaml.orig`。

---

## 阶段 3：协方差注入 Relay

### 3.1 症状

EKF 不崩溃了，但：
- `map → odom` TF 不发布（EKF 不收敛）
- EKF 输出诊断显示无法融合传感器数据

### 3.2 根因

通过 `ros2 topic echo /odom` 和 `/imu/data_raw` 发现 **所有协方差矩阵全为零**。

底盘驱动 `myslam_car_driver` 发布的里程计和 IMU 消息中 `covariance` 字段为全部 0。EKF 将零协方差解读为"传感器绝对精确"：
- Odom 声称"我绝对精确"（vx, vy, vyaw 完全可信）
- GPS 也声称"我绝对精确"
- 两者数据稍有矛盾 → EKF 无法收敛，拒绝发布 TF

### 3.3 修复

**新建文件**: `scripts/covariance_relay.py`

核心逻辑（完整代码见仓库）：

```python
class CovarianceRelay(Node):
    def __init__(self):
        super().__init__('covariance_relay')

        # 里程计协方差: 四轮差速, 低速场景
        self.odom_twist_cov = [
            0.0025, 0.0,    0.0,    0.0,    0.0,    0.0,   # vx  ~0.05 m/s
            0.0,    0.0004, 0.0,    0.0,    0.0,    0.0,   # vy  ~0.02 m/s
            0.0,    0.0,    1e-6,   0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    1e-6,   0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    1e-6,   0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.0025,  # vyaw ~0.05 rad/s
        ]
        self.odom_pose_cov = [
            0.01,  0.0,   0.0,   0.0,   0.0,   0.0,    # pose ~0.1m
            0.0,   0.01,  0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   1e-6,  0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   1e-6,  0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   1e-6,  0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.01,
        ]
        # IMU 陀螺仪协方差: ~0.01 rad/s 不确定度
        self.imu_gyro_cov = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]

    def _cb_odom(self, msg: Odometry):
        msg.pose.covariance = self.odom_pose_cov
        msg.twist.covariance = self.odom_twist_cov
        self._pub_odom.publish(msg)

    def _cb_imu(self, msg: Imu):
        msg.orientation_covariance = self.imu_orient_cov
        msg.angular_velocity_covariance = self.imu_gyro_cov
        msg.linear_acceleration_covariance = self.imu_accel_cov
        self._pub_imu.publish(msg)
```

发布话题：
- `/odom` → `/odom/relay`（带协方差）
- `/imu/data_raw` → `/imu/relay`（带协方差）

**修改文件**: `launch/bringup.launch.py`

在 EKF 之前（t=3s）启动 relay：

```python
TimerAction(period=3.0, actions=[
    ExecuteProcess(
        cmd=['python3',
             os.path.join(pkg, 'scripts', 'covariance_relay.py')],
        name='covariance_relay',
        output='screen',
    ),
]),
```

**修改文件**: `config/ekf.yaml`

EKF 改为订阅 relay 后的话题：

```yaml
odom0: /odom/relay    # 曾为 /odom
imu0:  /imu/relay     # 曾为 /imu/data_raw
```

### 3.4 效果

协方差注入后，EKF 开始正常发布 `map → odom` TF。第一个关键阻塞点解除。

---

## 阶段 4：缺失 TF 修复

### 4.1 症状

navsat_transform 反复报错：

```
[navsat_transform]: Could not obtain base_link -> gps transform.
Will not remove offset of navsat device from robot's origin.
```

### 4.2 根因

`navsat_transform_node` 需要 `base_link → gps` 静态变换来补偿 GPS 天线相对于机器人中心的偏移。系统中不存在此 TF。

### 4.3 修复

**修改文件**: `launch/bringup.launch.py`

添加静态变换发布器（立即启动，不等延迟）：

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['--x', '-0.14', '--y', '0.0', '--z', '0.12',
               '--roll', '0', '--pitch', '0', '--yaw', '0',
               '--frame-id', 'base_link', '--child-frame-id', 'gps'],
    name='gps_tf_publisher',
),
```

---

## 阶段 5：EKF 参数调优

### 5.1 症状

诊断探针（见阶段 7）显示：
- `/odometry/gps` 延迟 400-600ms
- EKF 日志：`Failed to meet update rate!`
- GPS 数据间歇性被丢弃

### 5.2 根因

1. `sensor_timeout: 0.1s`（100ms）太短，GPS 5-10Hz 的正常数据也被超时丢弃
2. `frequency: 20.0`（20Hz）Xavier NX 跑不动，实际只能跑到 ~7Hz
3. IMU 配置中融合了 `ax`（线性加速度），底盘振动噪声污染速度估计
4. 缺少 `process_noise_covariance` 和 `initial_estimate_covariance`，EKF 使用默认值（不适用于本系统）

### 5.3 修复

**文件**: `config/ekf.yaml`（多次迭代，当前最终版本）

```yaml
# 基础设置
frequency: 5.0              # 20→10→5Hz, Xavier NX 算力限制
sensor_timeout: 0.5         # 0.1→0.5s, 容忍 GPS 正常延迟

# 过程噪声协方差 (15x15 对角矩阵)
process_noise_covariance:
  [0.01, 0.0, ..., 0.01,   # x, y: 允许运动模型不确定性
   ...,
   0.005,                   # yaw: 运动模型 yaw 噪声
   ...,
   0.05, 0.05,             # vx, vy: 速度过程噪声
   ...,
   0.25, 0.25, 0.25]       # ax, ay, ayaw: 加速度过程噪声

# 初始估计协方差 (15x15 对角矩阵)
initial_estimate_covariance:
  [1.0, 1.0, ...,          # x, y: 初始位置不确定度 1m
   0.5,                     # yaw: 初始航向不确定度 0.7rad
   ...,
   0.5, 0.5, 0.5,          # vx, vy, vyaw 初始不确定度
   ...,
   1.0, 1.0, 1.0]           # 加速度初始不确定度

# 传感器配置
odom0: /odom/relay
odom0_config: [false,false,false, false,false,false,
               true, true, false,   # vx, vy    ← 只融合速度
               false,false,true,    # vyaw      ← 只融合角速度
               false,false,false]

imu0: /imu/relay
imu0_config: [false,false,false, false,false,false,
              false,false,false,
              false,false,true,     # vyaw only ← 只融合角速度Z
              false,false,false]    # 移除了 ax（加速度X），避免振动噪声

odom1: /odometry/gps
odom1_config: [true, true, false,  # x, y      ← 绝对位置
               false,false,true,   # yaw       ← 绝对航向 (最新修改)
               false,false,false,
               false,false,false,
               false,false,false]
```

---

## 阶段 6：navsat_transform 参数调优

### 6.1 修复

**文件**: `config/navsat.yaml`

```yaml
frequency: 5.0                    # 10→5Hz, 减少插值假数据
use_odometry_yaw: false           # true→false (关键修改!)
# 当 use_odometry_yaw=true 时:
#   /odometry/gps 方向 = EKF 自己的 yaw (纯死推算)
#   → EKF 没有绝对航向参考 → 代价地图旋转 → 螺旋畸变
#
# 当 use_odometry_yaw=false 时:
#   /odometry/gps 方向 = GPS 双天线航向 (绝对参考)
#   → EKF 融合后获得正确航向

yaw_offset: 1.5707963            # π/2, GPS 基线垂直于车头
zero_altitude: true
broadcast_cartesian_transform: true
```

---

## 阶段 7：诊断工具

### 7.1 `ekf_probe.py` — Python 诊断探针

**新建文件**: `scripts/ekf_probe.py`

功能：
- 监控 5 个关键话题的频率 / 延迟 / 消息数：`/odom`, `/imu/data_raw`, `/gps/fix`, `/odometry/gps`, `/cmd_vel`
- 检查 TF 树完整性：`map→odom`, `odom→base_link`, `base_link→livox_frame`
- 报告 GPS 质量（定位状态 + 协方差类型）
- 彩色终端输出（✅ 正常 / ⚠️ 警告 / 🔴 异常）
- 支持 `--rate`（报告间隔）、`--log`（日志文件）、`--topics-only`（简洁模式）

已知 Bug（已修复）：
- `/cmd_vel` 是 `Twist` 类型（无 `header` 字段），旧版直接访问 `msg.header.stamp` 导致 `AttributeError` 崩溃
- 修复：对 Twist 类型使用当前系统时间作为时间戳

### 7.2 `ekf_quick_check.sh` — Bash 快速检查

**新建文件**: `scripts/ekf_quick_check.sh`

功能：
- 使用 `ros2 topic hz` 和 `ros2 topic echo` 进行一次性快照
- 比 Python 探针更轻量，适合快速现场诊断
- ⚠️ 在 Xavier NX 上 `timeout` 命令不稳定，建议优先使用 Python 探针

### 7.3 `ekf_diagnostic.launch.py` — 诊断启动文件

**新建文件**: `launch/ekf_diagnostic.launch.py`

一键启动 Python 探针 + GPS 频率监控 + TF 频率监控。

---

## 阶段 8：Xavier NX CPU 过载调优

### 8.1 症状

以下警告在日志中密集出现：
```
[ekf_node]: Failed to meet update rate! Took 0.94s
[controller_server]: Control loop missed its desired rate of 20.0000Hz
[planner_server]: Planner loop missed its desired rate of 20.0000 Hz. Current loop rate is 0.89 Hz
[BehaviorTreeEngine]: Behavior Tree tick rate 100.00 was exceeded!
```

结果：
- EKF 掉频 → `map→odom` TF 不连续
- TF 不连续 → costmap 丢弃点云 (`Message Filter dropping message`)
- 点云被丢弃 → 代价地图消失
- 代价地图消失 + BT 超时 → 无法导航

### 8.2 根因

Xavier NX CPU 资源被 20+ 个节点竞争，全部节点的期望频率超出 CPU 能力：
- EKF: 10Hz（实际跑 2-5Hz）
- BT: 100Hz（`bt_loop_duration: 10ms`!）
- Controller: 20Hz
- Planner: 20Hz
- Behavior: 10Hz
- Velocity Smoother: 20Hz
- Costmap: 5Hz

### 8.3 修复

**文件**: `config/ekf.yaml`

```yaml
frequency: 5.0      # 10→5Hz
```

**文件**: `config/nav2_params.yaml`（大量修改）

```yaml
# 导航核心频率
controller_frequency: 10.0           # 20→10Hz
expected_planner_frequency: 5.0      # 20→5Hz
bt_loop_duration: 100                # 10→100ms (100Hz→10Hz) — 关键!
default_server_timeout: 60           # 20→60s (给 action server 更多响应时间)
cycle_frequency: 5.0                 # 10→5Hz (behavior)
smoothing_frequency: 10.0            # 20→10Hz
local_costmap.update_frequency: 2.0  # 5→2Hz

# TF 容忍度 — 关键修复!
transform_tolerance: 0.5             # 0.2→0.5s (全部6处)
# 原值 0.2s 时: EKF 掉频 > 0.2s → TF 缓存查不到 → 丢弃整个点云
# 新值 0.5s 时: 容忍 EKF 偶尔掉频, 点云不再被丢弃

# DWB 采样减半 (降低计算量)
vx_samples: 10                       # 20→10
vtheta_samples: 10                   # 20→10
sim_time: 1.5                        # 1.7→1.5s
```

### 8.4 效果

- EKF "Failed to meet update rate" 在稳态后消失（初始化阶段偶发）
- "Message Filter dropping message" 完全消失（transform_tolerance 修复生效）
- BT "tick rate 100.00 was exceeded" 大幅减少
- Controller/Planner loop 掉频警告大幅减少

---

## 阶段 9：GPS 航向融合

### 9.1 症状

即使修复了以上所有问题，导航时仍然出现：
- 代价地图旋转（前方障碍物在地图中出现在右侧）
- 机器人原地打转
- DWB 报告 `No valid trajectories out of 109! BaseObstacle/Trajectory Hits Obstacle`

### 9.2 根因分析

```
问题链:
  use_odometry_yaw: true (之前设置)
    → /odometry/gps 的方向 = EKF 自己的 yaw 值
      → EKF yaw 纯靠里程计/陀螺仪积分 (从 0 开始, 无绝对参考)
        → odom1_config 只融合 x, y 位置, 没有 yaw
          → EKF 永远不知道"北"在哪里
            → map 坐标系方向随缘
              → 代价地图与世界不对齐
                → 全局路径方向错误
                  → DWB 所有轨迹被判定为撞障碍物
                    → 原地打转
```

GPS 双天线虽然提供了绝对航向（基线 16cm，左右布局），但 `use_odometry_yaw: true` 让 navsat_transform **忽略了 GPS 航向**，改用 EKF 自己的死推算 yaw。而 EKF 又没有被配置为融合 GPS 航向（`odom1_config` yaw 位为 false），形成**双向死锁**。

### 9.3 修复

**文件**: `config/navsat.yaml`

```yaml
use_odometry_yaw: false   # true→false
# 改为 false 后, /odometry/gps 的方向来自 GPS 双天线航向 (绝对参考)
```

**文件**: `config/ekf.yaml`

```yaml
odom1: /odometry/gps
odom1_config: [true,  true,  false,   # x, y
               false, false, true,    # yaw ← 新增!
               false, false, false,
               false, false, false,
               false, false, false]
```

### 9.4 测试结果

测试中仍未完全解决——导航时代价地图旋转/打转问题依然存在。问题可能更深层，详见 [03_CURRENT_ISSUES.md](./03_CURRENT_ISSUES.md)。

---

## 修改文件汇总

| 文件 | 操作 | 关键变更 |
|------|------|----------|
| `CLAUDE.md` | 新建 | 项目 AI 助手指令 |
| `docs/architecture.svg` | 新建 | SVG 架构图 |
| `docs/architecture.md` | 新建 | 中文架构文档 |
| `docs/01_PROJECT_OVERVIEW.md` | 新建 | 项目概述（本文档集） |
| `docs/02_COLLABORATION_LOG.md` | 新建 | 本文档 |
| `docs/03_CURRENT_ISSUES.md` | 新建 | 当前问题记录 |
| `config/ekf.yaml` | 多次修改 | history_length, sensor_timeout, 频率, 过程噪声, GPS yaw 融合 |
| `config/ekf.yaml.orig` | 新建 | 初始配置备份 |
| `config/navsat.yaml` | 多次修改 | 频率, use_odometry_yaw |
| `config/navsat.yaml.orig` | 新建 | 初始配置备份 |
| `config/nav2_params.yaml` | 多次修改 | 全局降频, transform_tolerance, DWB 采样 |
| `launch/bringup.launch.py` | 多次修改 | 协方差 relay, GPS/LiDAR 静态 TF |
| `launch/ekf_diagnostic.launch.py` | 新建 | 诊断启动文件 |
| `scripts/covariance_relay.py` | 新建 | 协方差注入节点 |
| `scripts/ekf_probe.py` | 新建 | EKF 诊断探针 |
| `scripts/ekf_quick_check.sh` | 新建 | 快速诊断脚本 |
| `CMakeLists.txt` | 修改 | 添加 scripts 到 install 目标 |
