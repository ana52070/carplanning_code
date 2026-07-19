#!/usr/bin/env python3
import sys
import argparse
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class WaitForTF(Node):
    def __init__(self, parent_frame, child_frame, timeout_sec):
        super().__init__('wait_for_tf')
        self.parent = parent_frame
        self.child = child_frame
        self.timeout = timeout_sec
        self.done = False        # 用 flag 代替 sys.exit()
        self.exit_code = 0
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.create_timer(0.5, self._check)
        self.start = self.get_clock().now()
        self.get_logger().info(
            f'Waiting for TF: {parent_frame} → {child_frame} '
            f'(timeout={timeout_sec}s)')

    def _check(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9

        if elapsed > self.timeout:
            self.get_logger().error(
                f'Timeout after {self.timeout}s: '
                f'{self.parent} → {self.child} not found!')
            self.done = True
            self.exit_code = 1
            return

        try:
            self.tf_buf.lookup_transform(
                self.parent, self.child, rclpy.time.Time())
            self.get_logger().info(
                f'✅ TF ready: {self.parent} → {self.child} '
                f'(took {elapsed:.1f}s)')
            self.done = True
            self.exit_code = 0
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--parent', default='odom')
    parser.add_argument('--child', default='base_link')
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = WaitForTF(args.parent, args.child, args.timeout)

    # spin 直到 done flag 被设置
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)

    exit_code = node.exit_code
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)   # 正常退出，launch 能感知到


if __name__ == '__main__':
    main()
