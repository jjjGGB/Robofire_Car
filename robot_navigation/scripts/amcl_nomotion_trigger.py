#!/usr/bin/env python3
"""
AMCL No-Motion Update 自动触发器

问题背景:
  当 elevation_mapping 增加 CPU 负载时，FAST-LIO 的 TF 发布会滞后数百毫秒。
  AMCL 的 handleInitialPose 在 TF 查询失败时会 fallback 到单位矩阵并继续，
  但关键变量 latest_tf_valid_ 不会被设为 true。
  如果机器人静止，shouldUpdateFilter() 返回 false，导致 map→odom TF 永远不发布。

解决方案:
  监听 /initialpose 话题，在收到初始位姿后延迟调用 /request_nomotion_update 服务，
  强制 AMCL 在下一个激光扫描周期执行完整的粒子滤波更新。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty


class AmclNomotionTrigger(Node):
    def __init__(self):
        super().__init__('amcl_nomotion_trigger')

        # 延迟时间（秒）：等待 AMCL 处理完初始位姿后再触发
        self.declare_parameter('delay_sec', 1.0)
        self.delay_sec = self.get_parameter('delay_sec').value

        # 连续触发次数：确保至少一次成功
        self.declare_parameter('trigger_count', 3)
        self.trigger_count = self.get_parameter('trigger_count').value

        # 订阅初始位姿话题
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initial_pose,
            10
        )

        # 创建服务客户端
        self.client = self.create_client(Empty, '/request_nomotion_update')

        # 防止重复触发的定时器引用
        self._pending_timer = None

        self.get_logger().info(
            f'Waiting for /initialpose, will trigger nomotion update '
            f'after {self.delay_sec}s delay ({self.trigger_count} times)')

    def _on_initial_pose(self, msg):
        self.get_logger().info('Received initial pose, scheduling nomotion update trigger')
        # 取消之前的待执行定时器（防止重复触发）
        if self._pending_timer is not None:
            self._pending_timer.cancel()
        # 创建一次性延迟触发
        self._pending_timer = self.create_timer(
            self.delay_sec, self._trigger_nomotion_update)

    def _trigger_nomotion_update(self):
        """连续触发多次 nomotion update，确保 AMCL 完成初始化"""
        # 立即取消定时器，确保只执行一次
        if self._pending_timer is not None:
            self._pending_timer.cancel()
            self._pending_timer = None

        if not self.client.service_is_ready():
            self.get_logger().warn(
                '/request_nomotion_update service not available yet')
            return

        for i in range(self.trigger_count):
            req = Empty.Request()
            self.client.call_async(req)
            self.get_logger().info(
                f'Sent nomotion update request ({i + 1}/{self.trigger_count})')


def main(args=None):
    rclpy.init(args=args)
    node = AmclNomotionTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
