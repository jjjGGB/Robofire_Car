#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from four_wheel_steering_msgs.msg import FourWheelSteeringStamped


class SimOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__('sim_odom_bridge')

        self.declare_parameter('input_topic', '/odom_4ws')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('wheelbase', 2.071)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._wheelbase = float(self.get_parameter('wheelbase').value)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_stamp = None

        self._pub = self.create_publisher(Odometry, output_topic, 10)
        self._tf_pub = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            FourWheelSteeringStamped, input_topic, self._on_odom_4ws, 10)

        self.get_logger().info(
            f'Sim odom bridge started: {input_topic} -> {output_topic}, '
            f'{self._odom_frame} -> {self._base_frame}, wheelbase={self._wheelbase:.4f}')

    def _on_odom_4ws(self, msg: FourWheelSteeringStamped) -> None:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        current_time = stamp.sec + stamp.nanosec * 1e-9
        if self._last_stamp is None:
            self._last_stamp = current_time
        dt = max(0.0, current_time - self._last_stamp)
        self._last_stamp = current_time

        v = self._finite_or_zero(msg.data.speed)
        front = self._finite_or_zero(msg.data.front_steering_angle)
        rear = self._finite_or_zero(msg.data.rear_steering_angle)

        yaw_rate = 0.0
        if abs(self._wheelbase) > 1e-6:
            yaw_rate = v * (math.tan(front) - math.tan(rear)) / self._wheelbase

        if dt > 0.0:
            self._yaw += yaw_rate * dt
            self._x += v * math.cos(self._yaw) * dt
            self._y += v * math.sin(self._yaw) * dt

        qz = math.sin(self._yaw * 0.5)
        qw = math.cos(self._yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = yaw_rate
        self._pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header = odom.header
        tf_msg.child_frame_id = odom.child_frame_id
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z
        tf_msg.transform.rotation = odom.pose.pose.orientation
        self._tf_pub.sendTransform(tf_msg)

    @staticmethod
    def _finite_or_zero(value: float) -> float:
        return value if math.isfinite(value) else 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
