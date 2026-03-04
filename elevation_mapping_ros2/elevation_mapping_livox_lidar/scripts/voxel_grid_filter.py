#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs_py.point_cloud2 import create_cloud
import numpy as np


class VoxelGridFilter(Node):
    """
    Python实现的VoxelGrid点云下采样滤波器节点
    功能类似于ROS1的pcl_ros/VoxelGrid
    """

    def __init__(self):
        super().__init__('voxel_grid_filter')

        # 声明参数
        self.declare_parameter('leaf_size', 0.05)
        self.declare_parameter('filter_field_name', 'z')
        self.declare_parameter('filter_limit_min', 0.01)
        self.declare_parameter('filter_limit_max', 6.0)
        self.declare_parameter('filter_limit_negative', False)
        self.declare_parameter('input_topic', '/camera/depth/points')
        self.declare_parameter('output_topic', '/camera/depth/points_downsampled')

        # 获取参数
        self.leaf_size = self.get_parameter('leaf_size').value
        self.filter_field = self.get_parameter('filter_field_name').value
        self.filter_min = self.get_parameter('filter_limit_min').value
        self.filter_max = self.get_parameter('filter_limit_max').value
        self.filter_negative = self.get_parameter('filter_limit_negative').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # 创建订阅器和发布器
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )

        self.get_logger().info(
            f'VoxelGrid Filter 已启动\n'
            f'  输入话题: {input_topic}\n'
            f'  输出话题: {output_topic}\n'
            f'  体素大小: {self.leaf_size}m\n'
            f'  过滤字段: {self.filter_field}\n'
            f'  过滤范围: [{self.filter_min}, {self.filter_max}]'
        )

    def pointcloud_callback(self, msg):
        """处理输入点云并发布下采样结果"""
        try:
            # Skip messages with invalid timestamp
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                self.get_logger().warn('收到时间戳为0的点云，跳过')
                return

            # 将PointCloud2转换为numpy数组
            points = self.pointcloud2_to_array(msg)

            if points.size == 0:
                self.get_logger().warn('收到空点云')
                return

            # 应用距离滤波
            if self.filter_field in ['x', 'y', 'z']:
                field_idx = {'x': 0, 'y': 1, 'z': 2}[self.filter_field]
                mask = (points[:, field_idx] >= self.filter_min) & \
                       (points[:, field_idx] <= self.filter_max)
                if self.filter_negative:
                    mask = ~mask
                points = points[mask]

            # 应用体素网格下采样
            downsampled_points = self.voxel_grid_downsample(points, self.leaf_size)

            # 转换回PointCloud2并发布
            # Add small delay to timestamp to ensure TF is available
            from rclpy.time import Time, Duration
            original_time = Time.from_msg(msg.header.stamp)
            delayed_time = original_time + Duration(seconds=0.05)  # 50ms delay

            output_header = msg.header
            output_header.stamp = delayed_time.to_msg()
            output_msg = self.array_to_pointcloud2(downsampled_points, output_header)
            self.publisher.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f'点云处理失败: {str(e)}')

    def pointcloud2_to_array(self, cloud_msg):
        """将PointCloud2消息转换为Nx3 numpy数组 (x, y, z)"""
        points_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([point[0], point[1], point[2]])

        if len(points_list) == 0:
            return np.array([])

        return np.array(points_list, dtype=np.float32)

    def voxel_grid_downsample(self, points, leaf_size):
        """体素网格下采样"""
        if points.size == 0:
            return points

        # 计算体素索引
        voxel_indices = np.floor(points / leaf_size).astype(np.int32)

        # 使用字典存储每个体素中的点
        voxel_dict = {}
        for i, voxel_idx in enumerate(voxel_indices):
            key = tuple(voxel_idx)
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(points[i])

        # 对每个体素计算质心
        downsampled = []
        for voxel_points in voxel_dict.values():
            centroid = np.mean(voxel_points, axis=0)
            downsampled.append(centroid)

        return np.array(downsampled, dtype=np.float32)

    def array_to_pointcloud2(self, points, header):
        """将Nx3 numpy数组转换为PointCloud2消息"""
        if points.size == 0:
            # 返回空点云
            return PointCloud2(header=header)

        # 创建点云数据
        points_list = [tuple(point) for point in points]

        # 创建PointCloud2消息
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]

        return create_cloud(header, fields, points_list)


def main(args=None):
    rclpy.init(args=args)
    node = VoxelGridFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
