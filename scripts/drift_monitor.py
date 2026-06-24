#!/usr/bin/env python3
"""
=============================================================================
 导航漂移诊断脚本 — 持续监控导航过程中的关键指标，记录到 CSV
=============================================================================

用法:
    python3 drift_monitor.py                          # 屏幕输出 + CSV
    python3 drift_monitor.py --log /tmp/drift.csv     # 指定日志路径
    python3 drift_monitor.py --rate 1.0               # 每秒采样一次 (默认 0.5Hz)

监控指标:
    1.  LIO 里程计位姿 (odom→base_link TF) — 漂移检测
    2.  DWB 轨迹质量 — 有效轨迹比例
    3.  代价地图障碍物比例 — 是否异常
    4.  全局路径规划状态 — 是否有路径
    5.  /cmd_vel 输出 — 机器人是否在动
    6.  TF 健康 — 发布频率
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import signal
import sys
import time
import math
import csv
import os
from collections import deque
from datetime import datetime
from typing import Optional

import tf2_ros
import numpy as np

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage
from nav2_msgs.msg import Costmap


class DriftMonitor(Node):
    def __init__(self, rate: float = 0.5, csv_path: Optional[str] = None):
        super().__init__('drift_monitor')

        self.sample_interval = 1.0 / rate
        self.csv_path = csv_path or f'/tmp/drift_monitor_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.csv_file = None
        self.csv_writer = None
        self.sample_count = 0

        # ── 存储最新数据 ──────────────────────────────────
        self.latest_tf_odom_base: Optional[TransformStamped] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd_vel: Optional[Twist] = None
        self.latest_path: Optional[Path] = None

        # 初始位姿 (用于计算漂移量)
        self.init_x: Optional[float] = None
        self.init_y: Optional[float] = None
        self.init_yaw: Optional[float] = None

        # 历史数据 (用于检测变化率)
        self.tf_history = deque(maxlen=30)  # 最近30条TF记录
        self.yaw_history = deque(maxlen=30)

        # DWB 状态 (从 /rosout 或 topic 推断)
        self.cmd_vel_history = deque(maxlen=10)

        # 代价地图状态
        self.costmap_obstacle_ratio: float = -1.0

        # 错误计数
        self.tf_miss_count = 0
        self.dwb_no_valid_count = 0

        # ── 订阅 ──────────────────────────────────────────
        self._sub_odom = self.create_subscription(
            Odometry, '/fastlio2/lio_odom', self._cb_odom, 10)
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self._cb_cmd, 10)
        self._sub_path = self.create_subscription(
            Path, '/plan', self._cb_path, 10)
        self._sub_costmap = self.create_subscription(
            Costmap, '/local_costmap/costmap', self._cb_costmap, 10)

        # TF 广播监听
        self._sub_tf = self.create_subscription(
            TFMessage, '/tf', self._cb_tf, 100)

        # rosout 抓 DWB 报错
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        # 我们通过 /cmd_vel 是否为零来判断控制器状态

        # ── 定时器 ────────────────────────────────────────
        self._timer = self.create_timer(self.sample_interval, self._sample)
        self._start_time = time.time()

        # ── 初始化 CSV ────────────────────────────────────
        self._init_csv()

        self.get_logger().info(f"🔍 漂移监控已启动, 采样间隔 {self.sample_interval:.1f}s")
        self.get_logger().info(f"📝 CSV: {self.csv_path}")

    # ══════════════════════════════════════════════════════════
    #  回调
    # ══════════════════════════════════════════════════════════

    def _cb_odom(self, msg: Odometry):
        self.latest_odom = msg
        # 记录初始位姿
        if self.init_x is None:
            self.init_x = msg.pose.pose.position.x
            self.init_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            _, _, self.init_yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)

    def _cb_cmd(self, msg: Twist):
        self.latest_cmd_vel = msg
        self.cmd_vel_history.append(time.time())
        # 速度全零且之前有速度 → 可能是 DWB 失败
        if (abs(msg.linear.x) < 0.001 and abs(msg.angular.z) < 0.001
            and len(self.cmd_vel_history) > 2):
            # 检查最近是否有非零速度
            self.dwb_no_valid_count += 1

    def _cb_path(self, msg: Path):
        self.latest_path = msg

    def _cb_costmap(self, msg: Costmap):
        """计算代价地图中障碍物占比"""
        if len(msg.data) > 0:
            occupied = sum(1 for v in msg.data if v > 90)
            self.costmap_obstacle_ratio = occupied / len(msg.data)

    def _cb_tf(self, msg: TFMessage):
        for transform in msg.transforms:
            if (transform.header.frame_id == 'odom' and
                transform.child_frame_id == 'base_link'):
                self.latest_tf_odom_base = transform
                self.tf_history.append(transform)

                # 提取 yaw
                q = transform.transform.rotation
                _, _, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)
                self.yaw_history.append(yaw)
                break

    # ══════════════════════════════════════════════════════════
    #  采样 & 输出
    # ══════════════════════════════════════════════════════════

    def _init_csv(self):
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'elapsed_s', 'sample',
            # LIO pose
            'lio_x', 'lio_y', 'lio_z', 'lio_yaw_deg',
            # 漂移量 (相对于初始位姿)
            'drift_x_m', 'drift_y_m', 'drift_yaw_deg',
            # yaw 变化率 (最近窗口)
            'yaw_rate_window_deg_per_s',
            # 代价地图
            'costmap_obs_ratio',
            # 是否有全局路径
            'has_path',
            # cmd_vel
            'cmd_vx', 'cmd_vz',
            # 状态标记
            'is_moving', 'tf_age_ms',
        ])
        self.csv_file.flush()

    def _quat_to_euler(self, x, y, z, w):
        """四元数 → (roll, pitch, yaw)"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        return roll, pitch, yaw

    def _sample(self):
        now = time.time()
        elapsed = now - self._start_time
        self.sample_count += 1

        # ── 1. LIO 位姿 ──────────────────────────────────
        lio_x = lio_y = lio_z = lio_yaw = 0.0
        drift_x = drift_y = drift_yaw = 0.0
        tf_age = 999.0

        if self.latest_tf_odom_base is not None:
            t = self.latest_tf_odom_base
            lio_x = t.transform.translation.x
            lio_y = t.transform.translation.y
            lio_z = t.transform.translation.z
            _, _, lio_yaw = self._quat_to_euler(
                t.transform.rotation.x, t.transform.rotation.y,
                t.transform.rotation.z, t.transform.rotation.w)

            if self.init_x is not None:
                drift_x = lio_x - self.init_x
                drift_y = lio_y - self.init_y
                drift_yaw = math.degrees(
                    math.atan2(math.sin(lio_yaw - self.init_yaw),
                               math.cos(lio_yaw - self.init_yaw)))

            # TF 时间戳年龄
            tf_stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            tf_age = (now - tf_stamp) * 1000.0  # ms

        # ── 2. yaw 变化率 ────────────────────────────────
        yaw_rate = 0.0
        if len(self.yaw_history) >= 10:
            old_yaw = self.yaw_history[0]
            new_yaw = self.yaw_history[-1]
            # 处理环绕
            yaw_diff = math.atan2(math.sin(new_yaw - old_yaw),
                                  math.cos(new_yaw - old_yaw))
            time_span = (len(self.yaw_history) - 1) * 0.1  # 假设 ~10Hz
            if time_span > 0:
                yaw_rate = math.degrees(yaw_diff) / time_span

        # ── 3. cmd_vel ───────────────────────────────────
        cmd_vx = cmd_vz = 0.0
        is_moving = False
        if self.latest_cmd_vel is not None:
            cmd_vx = self.latest_cmd_vel.linear.x
            cmd_vz = self.latest_cmd_vel.angular.z
            is_moving = abs(cmd_vx) > 0.01 or abs(cmd_vz) > 0.01

        # ── 4. 路径 ──────────────────────────────────────
        has_path = self.latest_path is not None and len(self.latest_path.poses) > 0

        # ── 5. 输出 ──────────────────────────────────────
        row = [
            f"{elapsed:.1f}", self.sample_count,
            f"{lio_x:.4f}", f"{lio_y:.4f}", f"{lio_z:.4f}", f"{math.degrees(lio_yaw):.2f}",
            f"{drift_x:.4f}", f"{drift_y:.4f}", f"{drift_yaw:.2f}",
            f"{yaw_rate:.3f}",
            f"{self.costmap_obstacle_ratio:.4f}" if self.costmap_obstacle_ratio >= 0 else "N/A",
            "Y" if has_path else "N",
            f"{cmd_vx:.3f}", f"{cmd_vz:.3f}",
            "Y" if is_moving else "N",
            f"{tf_age:.0f}",
        ]

        self.csv_writer.writerow(row)
        self.csv_file.flush()

        # ── 屏幕输出 (精简) ──────────────────────────────
        status_icons = []
        if is_moving:
            status_icons.append("🚗")
        else:
            status_icons.append("⏸️")
        if has_path:
            status_icons.append("🛤️")
        else:
            status_icons.append("❌path")
        if self.costmap_obstacle_ratio > 0.05:
            status_icons.append("⚠️密集")
        if abs(yaw_rate) > 1.0:
            status_icons.append(f"🔄{yaw_rate:.1f}°/s")

        status_line = " ".join(status_icons)
        drift_info = (f"漂移: Δx={drift_x:+.3f}m Δy={drift_y:+.3f}m "
                      f"Δyaw={drift_yaw:+.1f}° | "
                      f"yaw变化率={yaw_rate:+.2f}°/s")
        cmd_info = f"cmd_vel: vx={cmd_vx:.2f} vz={cmd_vz:.2f}"
        costmap_info = (f"costmap障碍物={self.costmap_obstacle_ratio*100:.1f}%"
                        if self.costmap_obstacle_ratio >= 0 else "costmap: N/A")

        if self.sample_count % 10 == 1 or abs(yaw_rate) > 0.5:
            # 每10次或异常时打印详细信息
            print(f"\n[{elapsed:6.1f}s #{self.sample_count:4d}] {status_line}")
            print(f"  {drift_info}")
            print(f"  {cmd_info} | {costmap_info}")
            print(f"  LIO: x={lio_x:.3f} y={lio_y:.3f} yaw={math.degrees(lio_yaw):.1f}° "
                  f"| TF年龄={tf_age:.0f}ms")
            # 漂移告警
            if abs(yaw_rate) > 2.0:
                print(f"  ⚠️⚠️⚠️  yaw漂移速率过大 ({yaw_rate:.1f}°/s)! 代价地图正在旋转!")
            elif abs(yaw_rate) > 1.0:
                print(f"  ⚠️  yaw漂移速率偏高 ({yaw_rate:.1f}°/s)")
            if abs(drift_yaw) > 30:
                print(f"  🔴 累计yaw漂移超过30° ({drift_yaw:.1f}°)!")
        else:
            # 精简输出
            dots = "." * (self.sample_count % 5 + 1)
            print(f"\r[{elapsed:6.1f}s] {status_line} "
                  f"Δyaw={drift_yaw:+.1f}° yaw_rate={yaw_rate:+.2f}°/s{dots}",
                  end='', flush=True)

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="导航漂移诊断脚本")
    parser.add_argument('--log', type=str, default=None, help='CSV日志路径')
    parser.add_argument('--rate', type=float, default=0.5, help='采样频率 (Hz), 默认0.5')
    args = parser.parse_args()

    rclpy.init()
    node = DriftMonitor(rate=args.rate, csv_path=args.log)

    # 优雅退出
    def shutdown(sig, frame):
        node.get_logger().info("正在关闭...")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
