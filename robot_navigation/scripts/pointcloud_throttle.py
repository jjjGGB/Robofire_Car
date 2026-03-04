#!/usr/bin/env python3
"""
点云节流节点：以固定频率转发 PointCloud2 消息。

用途：降低 elevation_mapping 的点云输入频率，避免与 FAST-LIO 争抢 CPU。
原始 LiDAR 10Hz → 节流后 2Hz → elevation_mapping 处理负载降低 80%。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudThrottle(Node):
    def __init__(self):
        super().__init__('pointcloud_throttle')

        # 参数声明
        self.declare_parameter('input_topic', '/livox/lidar_PointCloud2')
        self.declare_parameter('output_topic', '/elevation/lidar_throttled')
        self.declare_parameter('rate', 2.0)  # Hz

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        rate = self.get_parameter('rate').value
        self.target_period_ns = int(1e9 / rate)
        self.last_pub_time = self.get_clock().now()

        # 使用 BEST_EFFORT + KEEP_LAST(1) 匹配 Gazebo 传感器 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self._on_pointcloud, qos)

        self.get_logger().info(
            f'Throttling [{input_topic}] -> [{output_topic}] @ {rate} Hz')

    def _on_pointcloud(self, msg: PointCloud2):
        now = self.get_clock().now()
        if (now - self.last_pub_time).nanoseconds >= self.target_period_ns:
            self.pub.publish(msg)
            self.last_pub_time = now


def main():
    rclpy.init()
    node = PointCloudThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
