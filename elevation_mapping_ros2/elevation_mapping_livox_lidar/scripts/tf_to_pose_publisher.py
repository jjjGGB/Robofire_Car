#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfToPosePublisher(Node):
    """Listens to a transform between from_frame and to_frame and publishes it
       as a pose with a zero covariance."""

    def __init__(self):
        super().__init__('tf_to_pose_publisher')

        # Declare and get parameters
        self.declare_parameter('from_frame', 'odom')
        self.declare_parameter('to_frame', 'base_footprint')

        self.from_frame = self.get_parameter('from_frame').get_parameter_value().string_value
        self.to_frame = self.get_parameter('to_frame').get_parameter_value().string_value
        pose_name = str(self.to_frame) + "_pose"

        # Initialize TF buffer and listener with larger cache time
        from rclpy.duration import Duration
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publisher
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            pose_name,
            10
        )

        # Create timer (20Hz = 0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'TF to Pose Publisher started. Publishing transform {self.from_frame} -> {self.to_frame} to {pose_name}'
        )

    def timer_callback(self):
        """Periodically lookup transform and publish as pose."""
        try:
            # Lookup transform
            trans = self.tf_buffer.lookup_transform(
                self.from_frame,
                self.to_frame,
                rclpy.time.Time()
            )

            # Create and fill pose message
            pose = PoseWithCovarianceStamped()
            pose.header = trans.header
            pose.pose.pose.position.x = trans.transform.translation.x
            pose.pose.pose.position.y = trans.transform.translation.y
            pose.pose.pose.position.z = trans.transform.translation.z
            pose.pose.pose.orientation = trans.transform.rotation

            # Zero covariance
            pose.pose.covariance = [0.0] * 36

            # Publish
            self.publisher.publish(pose)

        except TransformException as ex:
            # Silently ignore transform lookup failures
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TfToPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
