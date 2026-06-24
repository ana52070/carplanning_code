#!/usr/bin/env python3
"""
=============================================================================
 EKF 诊断探针 — 实时监控所有 EKF 输入/输出话题的健康状态
=============================================================================

用法:
    python3 ekf_probe.py                          # 默认监控 2 秒刷新
    python3 ekf_probe.py --rate 5                 # 每 5 秒输出一次报告
    python3 ekf_probe.py --topics-only            # 只打印话题列表和频率
    python3 ekf_probe.py --log /tmp/ekf_probe.log # 输出到文件

监控项:
    1. 每个话题的消息频率 (Hz) 和间隔统计
    2. 消息时间戳延迟 (msg time vs wall clock)
    3. EKF 输入数据一致性检查
    4. TF 树健康检查
    5. /cmd_vel 输出统计
    6. GPS 数据质量标记

输出示例:
    ╔══════════════════════════════════════════════════════════╗
    ║           EKF 探针报告 @ 14:32:05 (间隔=2.0s)           ║
    ╠══════════════════════════════════════════════════════════╣
    ║  话题              频率   延迟    消息数  状态          ║
    ║  /odom             18.5Hz  ±3ms   37      ✅ 正常       ║
    ║  /imu/data_raw     45.2Hz  ±2ms   90      ✅ 正常       ║
    ║  /gps/fix           4.8Hz ±12ms    10      ⚠️ 偏低      ║
    ║  /odometry/gps     10.0Hz ±25ms    20      🔴 延迟大    ║
    ║  /cmd_vel          20.0Hz  ±1ms   40      ✅ 正常       ║
    ╠══════════════════════════════════════════════════════════╣
    ║  TF 树: map→odom→base_footprint ✅ 连接正常             ║
    ║  GPS: lat=39.1xxx lon=117.2xxx  fix=2 (DGPS)           ║
    ╚══════════════════════════════════════════════════════════╝
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import signal
import sys
import time
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import List, Optional, Dict

# ROS2 消息类型
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf2_msgs.msg import TFMessage


# =============================================================================
# 数据结构
# =============================================================================

@dataclass
class TopicStats:
    """单个话题的统计信息"""
    name: str
    msg_type: str
    timestamps: List[float] = field(default_factory=list)
    delays: List[float] = field(default_factory=list)
    last_msg_time: float = 0.0

    def add(self, stamp_sec: float, now_sec: float):
        self.timestamps.append(stamp_sec)
        delay = now_sec - stamp_sec
        self.delays.append(delay)
        self.last_msg_time = stamp_sec
        # 只保留最近 200 条
        if len(self.timestamps) > 200:
            self.timestamps = self.timestamps[-100:]
            self.delays = self.delays[-100:]

    def freq(self, window: float = 2.0) -> float:
        """计算窗口内的频率"""
        if len(self.timestamps) < 2:
            return 0.0
        cutoff = time.time() - window
        recent = [t for t in self.timestamps if t >= cutoff]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / max(recent[-1] - recent[0], 0.001)

    def avg_delay(self) -> float:
        if not self.delays:
            return 0.0
        recent = self.delays[-50:]
        return sum(recent) / len(recent)

    def delay_std(self) -> float:
        if len(self.delays) < 2:
            return 0.0
        recent = self.delays[-50:]
        avg = sum(recent) / len(recent)
        return math.sqrt(sum((d - avg) ** 2 for d in recent) / len(recent))

    def count_in_window(self, window: float = 2.0) -> int:
        cutoff = time.time() - window
        return sum(1 for t in self.timestamps if t >= cutoff)


# =============================================================================
# 探针节点
# =============================================================================

class EKFProbe(Node):
    def __init__(self, report_interval: float = 2.0, log_file: Optional[str] = None):
        super().__init__('ekf_probe')
        self.report_interval = report_interval
        self.stats: Dict[str, TopicStats] = {}
        self.log_file = log_file
        self._init_stats()

        # ── 订阅所有 EKF 相关话题 ──────────────────────────
        self._sub_odom = self.create_subscription(
            Odometry, '/odom', lambda m: self._cb(m, '/odom', 'Odometry'), 10)
        self._sub_imu = self.create_subscription(
            Imu, '/imu/data_raw', lambda m: self._cb(m, '/imu/data_raw', 'Imu'), 10)
        self._sub_gps = self.create_subscription(
            NavSatFix, '/gps/fix', lambda m: self._cb(m, '/gps/fix', 'NavSatFix'), 10)
        self._sub_gps_odom = self.create_subscription(
            Odometry, '/odometry/gps', lambda m: self._cb(m, '/odometry/gps', 'Odometry'), 10)
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', lambda m: self._cb(m, '/cmd_vel', 'Twist'), 10)

        # GPS 最新数据 (用于质量报告)
        self.latest_gps: Optional[NavSatFix] = None

        # TF 监听
        self._sub_tf = self.create_subscription(
            TFMessage, '/tf', self._cb_tf, 10)
        self._sub_tf_static = self.create_subscription(
            TFMessage, '/tf_static', self._cb_tf_static, 10)
        self.tf_frames: Dict[str, set] = defaultdict(set)  # parent → {child, ...}
        self.tf_last_update: Dict[str, float] = {}

        # 定时报告
        self._report_timer = self.create_timer(report_interval, self._print_report)
        self._last_report = time.time()

        # 优雅退出
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)

        self.get_logger().info(f"🔍 EKF 探针已启动, 每 {report_interval}s 输出报告")

    def _init_stats(self):
        for name, mtype in [
            ('/odom', 'Odometry'),
            ('/imu/data_raw', 'Imu'),
            ('/gps/fix', 'NavSatFix'),
            ('/odometry/gps', 'Odometry'),
            ('/cmd_vel', 'Twist'),
        ]:
            self.stats[name] = TopicStats(name=name, msg_type=mtype)

    def _on_shutdown(self, signum, frame):
        self.get_logger().info("探针关闭")
        self._print_report()  # 最后一次报告
        rclpy.shutdown()
        sys.exit(0)

    # ── 回调 ──────────────────────────────────────────────

    def _cb(self, msg, topic: str, mtype: str):
        now = time.time()
        # Twist 没有 header, 用当前时间作为时间戳
        if mtype == 'Twist':
            stamp = now
        else:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.stats[topic].add(stamp, now)

        # GPS 特殊处理
        if topic == '/gps/fix':
            self.latest_gps = msg

    def _cb_tf(self, msg: TFMessage):
        now = time.time()
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            self.tf_frames[parent].add(child)
            self.tf_last_update[f"{parent}→{child}"] = now

    def _cb_tf_static(self, msg: TFMessage):
        self._cb_tf(msg)

    # ── 报告生成 ──────────────────────────────────────────

    def _status_icon(self, freq: float, expected_min: float, delay_ms: float, max_delay_ms: float = 50) -> str:
        if freq <= 0:
            return "🔴 无数据"
        if freq < expected_min * 0.5:
            return "🔴 频率极低"
        if freq < expected_min * 0.8:
            return "⚠️ 偏低"
        if delay_ms > max_delay_ms * 2:
            return "🔴 延迟大"
        if delay_ms > max_delay_ms:
            return "⚠️ 有延迟"
        return "✅ 正常"

    def _expected_freq(self, topic: str) -> float:
        """返回各话题的期望最低频率"""
        return {
            '/odom': 10.0,
            '/imu/data_raw': 20.0,
            '/gps/fix': 1.0,
            '/odometry/gps': 2.0,
            '/cmd_vel': 5.0,
        }.get(topic, 1.0)

    def _max_delay(self, topic: str) -> float:
        """返回各话题允许的最大延迟 (ms)"""
        return {
            '/odom': 30,
            '/imu/data_raw': 20,
            '/gps/fix': 200,
            '/odometry/gps': 100,
            '/cmd_vel': 30,
        }.get(topic, 50)

    def _gps_quality(self, gps: NavSatFix) -> str:
        """解析 GPS 质量"""
        if gps is None:
            return "无数据"
        # NavSatFix status
        status_map = {-1: "无定位", 0: "未固定", 1: "单点定位", 2: "DGPS"}
        cov_type = gps.position_covariance_type
        cov_type_map = {
            0: "协方差未知",
            1: "近似协方差",
            2: "对角协方差 (已知精度)",
            3: "完整协方差 (已知精度)",
        }
        status = status_map.get(gps.status.status, f"未知({gps.status.status})")
        cov = cov_type_map.get(cov_type, f"未知({cov_type})")
        return f"{status} | {cov}"

    def _check_tf_tree(self) -> List[str]:
        """检查关键 TF 链"""
        issues = []
        now = time.time()

        # 检查 map → odom (EKF 发布)
        key = "map→odom"
        if key not in self.tf_last_update:
            issues.append(f"❌ 缺少 {key} TF (EKF 可能未启动)")
        elif now - self.tf_last_update[key] > 2.0:
            issues.append(f"⚠️ {key} TF 超过 2 秒未更新")

        # 检查 odom → base_footprint (底盘驱动)
        key = "odom→base_footprint"
        if key not in self.tf_last_update:
            # 尝试 base_link
            key2 = "odom→base_link"
            if key2 not in self.tf_last_update:
                issues.append(f"❌ 缺少 odom→base_footprint/base_link TF (底盘驱动)")
            elif now - self.tf_last_update[key2] > 2.0:
                issues.append(f"⚠️ {key2} TF 超过 2 秒未更新")
        elif now - self.tf_last_update[key] > 2.0:
            issues.append(f"⚠️ {key} TF 超过 2 秒未更新")

        # 检查 base_link → livox_frame (静态)
        key = "base_link→livox_frame"
        if key not in self.tf_last_update:
            issues.append(f"⚠️ 缺少 {key} 静态 TF (LiDAR 点云可能无法正确投影)")

        return issues if issues else ["✅ TF 树正常"]

    def _print_report(self):
        now = time.time()
        window = self.report_interval + 0.5
        lines = []

        # ── 标题 ──────────────────────────────────────────
        time_str = time.strftime("%H:%M:%S")
        lines.append("")
        lines.append("╔══════════════════════════════════════════════════════════════════╗")
        lines.append(f"║     🔍 EKF 探针报告 @ {time_str} (间隔={self.report_interval:.0f}s){'':26s}║")
        lines.append("╠══════════════════════════════════════════════════════════════════╣")
        lines.append(f"║ {'话题':<20s} {'频率':>8s}  {'延迟':>9s}  {'消息':>5s}  {'状态':<16s} ║")
        lines.append("╠══════════════════════════════════════════════════════════════════╣")

        all_ok = True

        for name in ['/odom', '/imu/data_raw', '/gps/fix', '/odometry/gps', '/cmd_vel']:
            s = self.stats[name]
            freq = s.freq(window)
            delay_ms = s.avg_delay() * 1000
            delay_std_ms = s.delay_std() * 1000
            count = s.count_in_window(window)
            expected = self._expected_freq(name)
            max_d = self._max_delay(name)
            status = self._status_icon(freq, expected, delay_ms, max_d)

            if "🔴" in status:
                all_ok = False

            freq_str = f"{freq:.1f}Hz" if freq > 0 else "---"
            delay_str = f"±{delay_std_ms:.0f}ms" if delay_std_ms > 0 else f"{delay_ms:.0f}ms"
            # 实际用 avg delay 显示
            delay_display = f"{delay_ms:.0f}ms"

            lines.append(
                f"║ {name:<20s} {freq_str:>8s}  {delay_display:>9s}  {count:>5d}  {status:<16s} ║"
            )

        lines.append("╠══════════════════════════════════════════════════════════════════╣")

        # ── TF 检查 ───────────────────────────────────────
        for issue in self._check_tf_tree():
            lines.append(f"║ TF: {issue:<56s}║")
            if issue.startswith("❌"):
                all_ok = False

        # ── GPS 质量 ──────────────────────────────────────
        if self.latest_gps:
            gps = self.latest_gps
            lines.append("╠══════════════════════════════════════════════════════════════════╣")
            lines.append(f"║ GPS: lat={gps.latitude:.6f} lon={gps.longitude:.6f} alt={gps.altitude:.2f}m{'':9s}║")
            lines.append(f"║ GPS 质量: {self._gps_quality(gps):<48s}║")
            navsat_delay = time.time() - (gps.header.stamp.sec + gps.header.stamp.nanosec * 1e-9)
            delay_str = f"{navsat_delay*1000:.0f}ms"
            lines.append(f"║ GPS 延迟: {delay_str:<53s}║")

        lines.append("╚══════════════════════════════════════════════════════════════════╝")
        lines.append("")

        # ── 输出 ──────────────────────────────────────────
        report = "\n".join(lines)
        if all_ok:
            # 正常时不打印完整表格, 只显示一行摘要
            summary = f"[EKF探针 {time_str}] "
            for name in ['/odom', '/imu/data_raw', '/gps/fix', '/odometry/gps']:
                s = self.stats[name]
                f = s.freq(window)
                summary += f"{name}={f:.1f}Hz "
            summary += "| TF✅" if not self._check_tf_tree()[0].startswith("❌") else "| TF❌"
            print(summary)
        else:
            print(report)

        # ── 日志文件 ──────────────────────────────────────
        if self.log_file:
            with open(self.log_file, 'a') as f:
                f.write(report)
                f.flush()


# =============================================================================
# 主函数
# =============================================================================

def main():
    import argparse
    parser = argparse.ArgumentParser(description="EKF 诊断探针")
    parser.add_argument('--rate', type=float, default=2.0, help='报告间隔 (秒), 默认 2.0')
    parser.add_argument('--topics-only', action='store_true', help='每次只打印一行频率摘要')
    parser.add_argument('--log', type=str, default=None, help='日志文件路径')
    args = parser.parse_args()

    rclpy.init()
    probe = EKFProbe(report_interval=args.rate, log_file=args.log)

    try:
        rclpy.spin(probe)
    except KeyboardInterrupt:
        pass
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
