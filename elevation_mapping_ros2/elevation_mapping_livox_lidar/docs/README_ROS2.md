# Elevation Mapping Demos - ROS2

这是从 ROS1 移植到 ROS2 的 elevation_mapping_livox_lidar 仿真演示包。

## 已移植的内容

### ✅ 已完成的移植

1. **包结构**
   - `package.xml`: catkin → ament_cmake (format 3)
   - `CMakeLists.txt`: catkin → ament_cmake
   - 移除 `COLCON_IGNORE` 文件

2. **Python 脚本**
   - `tf_to_pose_publisher.py`: rospy → rclpy
   - 采用面向对象的 ROS2 节点设计

3. **Launch 文件** (XML → Python)
   - `simple_demo.launch.py` - 简单点云演示
   - `turtlebot3_waffle_demo.launch.py` - TurtleBot3 Gazebo 仿真
   - `visualization.launch.py` - 可视化节点

4. **配置文件**
   - 所有配置文件更新为 ROS2 参数格式 (`ros__parameters`)
   - `waffle_robot.yaml`
   - `simple_demo_map.yaml`
   - `visualization/fused.yaml`
   - `visualization/raw.yaml`

## 使用方法

### 1. 编译

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select elevation_mapping_livox_lidar --symlink-install
source install/setup.bash
```

### 2. 运行演示

#### 简单演示 (Simple Demo)

发布预录制的点云数据并生成高程地图：

```bash
ros2 launch elevation_mapping_livox_lidar simple_demo.launch.py
```

这个演示会：
- 启动 elevation_mapping 节点
- 发布一个迷宫场景的点云数据 (maze.ply)
- 启动可视化节点
- 打开 RViz2 显示高程地图

#### TurtleBot3 Waffle Gazebo 仿真

在 Gazebo 中启动 TurtleBot3 机器人并进行高程建图：

```bash
# 设置 TurtleBot3 模型环境变量
export TURTLEBOT3_MODEL=waffle

# 启动仿真
ros2 launch elevation_mapping_livox_lidar turtlebot3_waffle_demo.launch.py
```

可选参数：
```bash
ros2 launch elevation_mapping_livox_lidar turtlebot3_waffle_demo.launch.py \
    x_pos:=-2.0 \
    y_pos:=1.5 \
    z_pos:=0.0
```

这个演示会：
- 启动 Gazebo 仿真环境（TurtleBot3 House 世界）
- 生成 TurtleBot3 Waffle 机器人
- 启动点云下采样滤波器（VoxelGrid）
- 启动 elevation_mapping 节点
- 打开 RViz2 显示高程地图

### 3. 控制机器人

在 TurtleBot3 演示中，你可以使用键盘或其他方式控制机器人移动：

```bash
# 键盘控制
ros2 run turtlebot3_teleop teleop_keyboard

# 或使用导航功能（需要额外配置）
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

## 主要变更说明

### ROS1 → ROS2 差异

| 方面 | ROS1 | ROS2 |
|------|------|------|
| Launch 格式 | XML | Python |
| Python API | rospy | rclpy |
| 节点设计 | 函数式 | 面向对象 (继承 Node) |
| 参数格式 | YAML 扁平结构 | `ros__parameters` 嵌套 |
| 可视化工具 | rviz | rviz2 |
| Nodelet | nodelet | 组件 (components) |
| 静态 TF | args 格式 | 列表参数格式 |

### 点云滤波器

**重要变更**：ROS1 中使用 `pcl/VoxelGrid` nodelet 进行点云下采样。ROS2 Humble 的 `pcl_ros` 包不提供独立的 VoxelGrid 可执行文件（源码存在但未完成 ROS2 移植）。

**解决方案**：实现了**自定义 Python VoxelGrid 滤波节点** (`voxel_grid_filter.py`)，提供以下功能：
- 体素网格下采样（可配置体素大小）
- 距离范围过滤（基于 x/y/z 字段）
- 与原 pcl_ros 兼容的参数接口

**可配置参数：**
```yaml
leaf_size: 0.05                    # 体素大小（米）
filter_field_name: 'z'              # 过滤字段
filter_limit_min: 0.01              # 最小距离
filter_limit_max: 6.0               # 最大距离
filter_limit_negative: false        # 反向过滤
input_topic: '/camera/depth/points'
output_topic: '/camera/depth/points_downsampled'
```

### 参数配置

所有 YAML 配置文件现在使用标准的 ROS2 格式：

```yaml
node_name:
  ros__parameters:
    parameter_name: value
```

## 依赖项

确保已安装以下 ROS2 包：

```bash
sudo apt install \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-robot-state-publisher \
    ros-humble-pcl-ros \
    ros-humble-tf2-ros
```

## 故障排除

### 问题：找不到 elevation_mapping

**解决方案**：确保先编译 elevation_mapping 包：
```bash
colcon build --packages-select elevation_mapping
```

### 问题：Gazebo 启动失败

**解决方案**：检查 TURTLEBOT3_MODEL 环境变量：
```bash
export TURTLEBOT3_MODEL=waffle
```

### 问题：点云滤波器相关错误

**说明**：ROS2 Humble 的 `pcl_ros` 包不提供独立的 `voxel_grid` 可执行文件。

**解决方案**：本包已实现自定义 Python VoxelGrid 滤波节点 (`voxel_grid_filter.py`)，会自动在 launch 文件中启动。如遇到滤波器错误，请检查：

```bash
# 检查节点是否安装
ros2 pkg executables elevation_mapping_livox_lidar

# 应该看到：
# elevation_mapping_livox_lidar tf_to_pose_publisher.py
# elevation_mapping_livox_lidar voxel_grid_filter.py
```

如果没有看到，重新编译：
```bash
colcon build --packages-select elevation_mapping_livox_lidar --symlink-install
```

### 问题：/camera/depth/points 话题没有数据

**问题原因**：TurtleBot3 Waffle 的默认 Gazebo 模型只包含 RGB 相机插件，没有深度相机/点云发布插件。

**解决方案**：本包提供了修改版的 URDF 文件 (`urdf/turtlebot3_waffle_with_depthcam.urdf.xacro`)，该文件包含：
- 基础 TurtleBot3 Waffle URDF（通过 xacro:include 引入）
- 深度相机传感器（type="depth"，绑定到 camera_rgb_frame）
- 点云发布功能（`/camera/depth/points` 话题）
- 深度图像和 RGB 图像话题
- Gazebo 差速驱动控制插件（发布 odom -> base_footprint TF）
- 关节状态发布插件

**架构说明：**
- robot_state_publisher：从 URDF 发布静态 TF（机器人链接关系）
- Gazebo diff_drive 插件：发布动态 TF（odom -> base_footprint）
- 所有 TF 形成统一完整的树，无断开问题

launch 文件会自动使用这个 URDF 模型，无需额外配置。

**验证点云发布：**
```bash
# 启动仿真后，检查话题
ros2 topic list | grep camera

# 应该看到：
# /camera/depth/camera_info
# /camera/depth/image_raw
# /camera/depth/points
# /camera/rgb/camera_info
# /camera/rgb/image_raw

# 查看点云数据
ros2 topic echo /camera/depth/points --once
```

### 问题：TF 树断开，elevation_mapping 一直等待 TF

**问题原因**：之前可能同时使用 SDF 文件和 robot_state_publisher 会导致产生两个独立的 TF 树。

**解决方案**：现已统一使用 URDF + robot_state_publisher + Gazebo 插件方案：
- robot_state_publisher 从 URDF 发布静态 TF（机器人关节）
- Gazebo diff_drive 插件发布 odom -> base_footprint 的动态 TF
- 所有 TF 形成完整的树：odom -> base_footprint -> base_link -> ... -> camera_rgb_optical_frame

**验证 TF 树：**
```bash
# 检查 TF 树结构
ros2 run tf2_tools view_frames

# 或者查看特定变换
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 问题：RViz2 配置文件错误

**解决方案**：如果 RViz2 配置文件是 ROS1 格式，可能需要手动调整。可以先不加载配置文件启动：
```bash
ros2 launch elevation_mapping_livox_lidar simple_demo.launch.py
# 然后在 RViz2 中手动添加所需的显示项
```

## 未移植的内容

以下 launch 文件未移植（非仿真相关）：
- `starleth_kinect_demo.launch.xml` - 需要特定硬件
- `ground_truth_demo.launch.xml` - 依赖特定传感器
- `realsense_demo.launch.xml` - 需要 RealSense 相机

如需这些功能，请参考移植模式自行转换。

## 贡献

移植完成日期：2026-01-25
ROS2 版本：Humble

## 许可证

BSD License (与原项目相同)
