#!/usr/bin/env python3
"""
=============================================================================
 协方差注入 Relay — 修复底盘驱动不输出协方差的问题
=============================================================================

myslam_car_driver 发布的 /odom 和 /imu/data_raw 协方差矩阵全是 0,
导致 EKF 认为这些测量"绝对精确", 无法正确融合 GPS 修正。

本节点订阅原始话题, 填充合理的协方差值后重新发布, EKF 订阅 relay 后的话题。

发布的话题:
    /odom/relay        — 带协方差的里程计
    /imu/relay         — 带协方差的 IMU

用法:
    python3 covariance_relay.py
=============================================================================
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np


class CovarianceRelay(Node):
    def __init__(self):
        super().__init__('covariance_relay')

        # ── 里程计协方差设定 ──────────────────────────────
        # 底盘是四轮差速, 低速校园场景:
        #   vx  ~0.05 m/s 不确定度 → 0.0025 (m/s)²
        #   vy  ~0.02 m/s 不确定度 → 0.0004
        #   vyaw ~0.05 rad/s 不确定度 → 0.0025
        #   (pose 我们不融合, 但填上免得报错)
        self.odom_twist_cov = [
            0.0025, 0.0,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.0004, 0.0,    0.0,    0.0,    0.0,
            0.0,    0.0,    1e-6,   0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    1e-6,   0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    1e-6,   0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.0025,
        ]
        self.odom_pose_cov = [
            0.01,  0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.01,  0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   1e-6,  0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   1e-6,  0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   1e-6,  0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.01,
        ]

        # ── IMU 协方差设定 ─────────────────────────────────
        # 陀螺仪 (角速度): ~0.01 rad/s 不确定度 → 0.0001
        # 加速度计: 我们不融合, 但填一下
        self.imu_gyro_cov = [
            0.0001, 0.0,    0.0,
            0.0,    0.0001, 0.0,
            0.0,    0.0,    0.0001,
        ]
        self.imu_accel_cov = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01,
        ]
        self.imu_orient_cov = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01,
        ]

        # ── 订阅 ───────────────────────────────────────────
        self._sub_odom = self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)
        self._sub_imu = self.create_subscription(
            Imu, '/imu/data_raw', self._cb_imu, 10)

        # ── 发布 ───────────────────────────────────────────
        self._pub_odom = self.create_publisher(Odometry, '/odom/relay', 10)
        self._pub_imu = self.create_publisher(Imu, '/imu/relay', 10)

        self.get_logger().info('协方差注入 Relay 已启动')
        self.get_logger().info('  /odom → /odom/relay (带协方差)')
        self.get_logger().info('  /imu/data_raw → /imu/relay (带协方差)')

    def _cb_odom(self, msg: Odometry):
        msg.pose.covariance = self.odom_pose_cov
        msg.twist.covariance = self.odom_twist_cov
        self._pub_odom.publish(msg)

    def _cb_imu(self, msg: Imu):
        msg.orientation_covariance = self.imu_orient_cov
        msg.angular_velocity_covariance = self.imu_gyro_cov
        msg.linear_acceleration_covariance = self.imu_accel_cov
        self._pub_imu.publish(msg)


def main():
    rclpy.init()
    node = CovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Relay 异常: {e}')
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
