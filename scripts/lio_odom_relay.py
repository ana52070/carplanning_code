#!/usr/bin/env python3
"""
lio_odom_relay.py — FAST-LIO2 里程计协方差注入节点

FAST-LIO2 (liangheming 版本) 输出的 Odometry 协方差矩阵全零,
导致 EKF 认为 LIO 是"无限精确"的, GPS 修正完全失效.

本节点:
1. 订阅 /fastlio2/lio_odom
2. 注入合理的协方差值
3. 发布到 /lio_odom/relay

协方差取值依据:
  FAST-LIO2 位置精度 ~1cm  → pose cov 0.001 (保守)
  FAST-LIO2 yaw 精度 ~0.1° → pose yaw cov 0.0003
  FAST-LIO2 速度精度 ~1cm/s → twist cov 0.001
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class LioOdomRelay(Node):
    def __init__(self):
        super().__init__('lio_odom_relay')

        self.sub = self.create_subscription(
            Odometry,
            '/fastlio2/lio_odom',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            Odometry,
            '/lio_odom/relay',
            10
        )

        self.get_logger().info('LIO Odom Relay started: /fastlio2/lio_odom → /lio_odom/relay')

    def callback(self, msg: Odometry):
        # ── 位姿协方差 (6x6 行优先, 36 元素) ──
        # 对角线索引: [0]=x, [7]=y, [14]=z, [21]=roll, [28]=pitch, [35]=yaw
        pose_cov = list(msg.pose.covariance)
        pose_cov[0]  = 0.001     # x  方差 (1cm² 量级, 保守给 0.001)
        pose_cov[7]  = 0.001     # y
        pose_cov[14] = 0.01      # z  (2D 模式不太关心, 给大一点)
        pose_cov[21] = 0.01      # roll  (2D 模式不太关心)
        pose_cov[28] = 0.01      # pitch (2D 模式不太关心)
        pose_cov[35] = 0.0003    # yaw (0.1° ≈ 0.00175 rad, 方差 ≈ 3e-4)
        msg.pose.covariance = pose_cov

        # ── 速度协方差 (6x6 行优先, 36 元素) ──
        twist_cov = list(msg.twist.covariance)
        twist_cov[0]  = 0.001    # vx
        twist_cov[7]  = 0.001    # vy
        twist_cov[14] = 0.01     # vz
        twist_cov[21] = 0.01     # wx (roll rate)
        twist_cov[28] = 0.01     # wy (pitch rate)
        twist_cov[35] = 0.001    # wz (yaw rate)
        msg.twist.covariance = twist_cov

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
