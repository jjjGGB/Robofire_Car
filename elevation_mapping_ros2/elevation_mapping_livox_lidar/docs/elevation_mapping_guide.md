# Elevation Mapping 高程图系统技术文档

## 一、系统概述

Elevation Mapping 是一个基于 ROS 2 的实时高程地图构建系统，最初由 ETH Zurich 的 ANYbotics 团队开发。该系统能够从深度相机或激光雷达等传感器获取点云数据，并通过贝叶斯融合算法构建连续更新的 2.5D 高程地图。

### 1.1 核心特性

- **多传感器支持**：激光雷达、深度相机、结构光传感器
- **贝叶斯融合**：基于卡尔曼滤波的高度估计与不确定性追踪
- **机器人运动补偿**：利用里程计协方差更新地图不确定性
- **可见性清理**：通过射线追踪移除动态障碍物
- **后处理管道**：支持法向量计算、孔洞填充、通行性分析等

---

## 二、系统架构

### 2.1 数据流图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Elevation Mapping 数据流                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   传感器点云 (/livox/lidar_PointCloud2)                                  │
│        │                                                                 │
│        ▼                                                                 │
│   ┌─────────────────────┐                                               │
│   │  InputSourceManager │ ← 管理多个输入源，支持异构传感器融合            │
│   └──────────┬──────────┘                                               │
│              │                                                           │
│              ▼                                                           │
│   ┌─────────────────────┐     ┌─────────────────────┐                   │
│   │   SensorProcessor   │ ←── │  LaserSensorProcessor │                 │
│   │   (传感器处理器)     │     │  (激光雷达噪声模型)    │                 │
│   └──────────┬──────────┘     └─────────────────────┘                   │
│              │                                                           │
│              │  处理流程:                                                │
│              │  1. 点云滤波 (NaN去除、体素网格)                          │
│              │  2. 坐标变换 (livox_frame → odom)                        │
│              │  3. 方差计算 (基于距离的噪声模型)                         │
│              │                                                           │
│              ▼                                                           │
│   ┌─────────────────────┐                                               │
│   │   ElevationMap      │ ← 核心地图类                                  │
│   │   (高程地图)         │                                               │
│   └──────────┬──────────┘                                               │
│              │                                                           │
│              │  融合算法:                                                │
│              │  1. 网格索引查询                                          │
│              │  2. Mahalanobis 距离检测                                  │
│              │  3. 卡尔曼滤波更新                                        │
│              │                                                           │
│              ▼                                                           │
│   ┌─────────────────────┐     ┌─────────────────────┐                   │
│   │ RobotMotionUpdater  │ ←── │    TF2 / Odometry    │                  │
│   │ (机器人运动补偿)     │     │    (位姿与协方差)     │                  │
│   └──────────┬──────────┘     └─────────────────────┘                   │
│              │                                                           │
│              ▼                                                           │
│   ┌─────────────────────┐                                               │
│   │  Visibility Cleanup │ ← 射线追踪算法移除遮挡点                       │
│   │  (可见性清理)        │                                               │
│   └──────────┬──────────┘                                               │
│              │                                                           │
│              ▼                                                           │
│   ┌─────────────────────┐                                               │
│   │  Postprocessor      │ ← 后处理滤波器管道                             │
│   │  Pipeline           │   (法向量、坡度、通行性)                        │
│   └──────────┬──────────┘                                               │
│              │                                                           │
│              ▼                                                           │
│   发布 GridMap (/elevation_mapping/elevation_map)                        │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 核心类结构

| 类名 | 文件位置 | 职责 |
|------|----------|------|
| `ElevationMapping` | `ElevationMapping.cpp` | 主控制器，管理订阅、服务、定时器 |
| `ElevationMap` | `ElevationMap.cpp` | 地图数据结构与融合算法 |
| `InputSourceManager` | `InputSourceManager.cpp` | 多输入源管理与调度 |
| `SensorProcessorBase` | `SensorProcessorBase.cpp` | 传感器处理器基类 |
| `LaserSensorProcessor` | `LaserSensorProcessor.cpp` | 激光雷达噪声模型 |
| `RobotMotionMapUpdater` | `RobotMotionMapUpdater.cpp` | 机器人运动引起的不确定性更新 |

---

## 三、核心算法详解

### 3.1 激光雷达噪声模型

**文件**: `LaserSensorProcessor.cpp:52-108`

激光雷达采用各向异性噪声模型，参考论文：
> Pomerleau et al., "Noise characterization of depth sensors for surface inspections," CARPI 2012.

```cpp
// 噪声模型参数
σ_beam = min_radius                           // 光束方向标准差（测距精度）
σ_lateral = beam_constant + beam_angle × d    // 横向标准差（随距离增加）

// 传感器协方差矩阵
Σ_sensor = diag(σ_lateral², σ_lateral², σ_beam²)
```

**参数说明**:

| 参数 | Livox Mid360 推荐值 | 说明 |
|------|---------------------|------|
| `min_radius` | 0.02 | 测距精度 ~2cm |
| `beam_angle` | 0.0003 | 角度分辨率贡献 |
| `beam_constant` | 0.005 | 固定横向误差 |

### 3.2 高度融合算法

**文件**: `ElevationMap.cpp:101-240`

采用递归贝叶斯估计（卡尔曼滤波）融合多次测量：

```cpp
// 新测量: z_new, 方差: σ²_new
// 当前估计: z_old, 方差: σ²_old

// 卡尔曼增益
K = σ²_old / (σ²_old + σ²_new)

// 融合高度
z_fused = z_old + K × (z_new - z_old)

// 融合方差（不确定性降低）
σ²_fused = (1 - K) × σ²_old
```

### 3.3 多高度检测

用于处理阶梯、悬崖等场景：

```cpp
// Mahalanobis 距离检测
d_mahal = |z_new - z_old| / √(σ²_old + σ²_new)

if (d_mahal > mahalanobis_distance_threshold) {
    // 检测到高度突变，可能是新表面
    // 根据时间戳决定保留新值或旧值
}
```

### 3.4 机器人运动补偿

**文件**: `RobotMotionMapUpdater.cpp`

机器人移动会增加地图的不确定性：

```cpp
// 从里程计获取位姿协方差 Σ_pose (6×6)
// 简化为 4×4 矩阵 (x, y, z, yaw)

// 计算相对运动协方差
Σ_relative = computeRelativePoseCovariance(pose_new, pose_old)

// 更新地图方差
σ²_map_new = σ²_map_old + covariance_scale × Σ_relative
```

---

## 四、Livox Mid360 激光雷达适配

### 4.1 问题分析

Livox 仿真插件发布两个话题：

| 话题 | 消息类型 | 兼容性 |
|------|----------|--------|
| `/livox/lidar` | `livox_ros_driver2::msg::CustomMsg` | ❌ 自定义格式 |
| `/livox/lidar_PointCloud2` | `sensor_msgs::msg::PointCloud2` | ✅ 标准格式 |

**关键代码** (`livox_points_plugin.cpp:75-77`):
```cpp
cloud2_pub = node_->create_publisher<PointCloud2>(topic + "_PointCloud2", 10);
custom_pub = node_->create_publisher<CustomMsg>(topic, 10);
```

### 4.2 TF 树结构

```
odom
 └── base_footprint
      └── base_link
           ├── imu_link
           ├── livox_frame    ← 激光雷达坐标系
           ├── wheel_1
           ├── wheel_2
           ├── wheel_3
           └── wheel_4
```

### 4.3 里程计时间同步问题

**问题现象**:
```
[ERROR] Could not get pose information from robot for time 514.546. Buffer empty?
```

**原因分析**:
1. 默认参数 `robot_odom_topic` 为 `/odom`
2. 里程计缓存与点云时间戳不对齐
3. `robotOdomCache_.getElemBeforeTime()` 找不到匹配数据

**解决方案**:
设置 `robot_odom_topic: ""` 禁用里程计订阅，直接使用 TF 获取位姿。

---

## 五、配置文件说明

### 5.1 文件结构

```
gz_simulation/config/elevation_mapping/
├── livox_robot.yaml              # 机器人与传感器配置
├── livox_map.yaml                # 地图参数配置
├── postprocessor_pipeline.yaml   # 后处理滤波器
└── livox_elevation_mapping.rviz  # RViz 可视化配置
```

### 5.2 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `map_frame_id` | `odom` | 地图坐标系 |
| `robot_base_frame_id` | `base_footprint` | 机器人基座坐标系 |
| `robot_odom_topic` | `""` | 空=禁用里程计，使用TF |
| `resolution` | `0.05` | 栅格分辨率 5cm |
| `length_in_x/y` | `8.0` | 地图尺寸 8m×8m |

---

## 六、使用方法

### 6.1 编译

```bash
cd /home/complexity/robot_ws
colcon build --packages-select gz_simulation
source install/setup.bash
```

### 6.2 启动

**方式一：联合启动**
```bash
ros2 launch gz_simulation simulation_with_elevation_mapping.launch.py
```

**方式二：分步启动**
```bash
# 终端1: 仿真环境
ros2 launch gz_simulation simulation.launch.py

# 终端2: 高程图（等待15秒后）
ros2 launch gz_simulation livox_elevation_mapping.launch.py
```

### 6.3 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/livox/lidar_PointCloud2` | PointCloud2 | 输入 | Livox 点云 |
| `/odom` | Odometry | 输入 | 里程计 |
| `/elevation_mapping/elevation_map` | GridMap | 输出 | 融合地图 |
| `/elevation_mapping/elevation_map_raw` | GridMap | 输出 | 原始地图 |

### 6.4 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/elevation_mapping/clear_map` | Empty | 清空地图 |
| `/elevation_mapping/save_map` | ProcessFile | 保存地图 |
| `/elevation_mapping/load_map` | ProcessFile | 加载地图 |

---

## 七、调优建议

### 7.1 地图质量优化

| 场景 | 参数调整 |
|------|----------|
| 精度优先 | 降低 `resolution` (如 0.03) |
| 性能优先 | 提高 `resolution` (如 0.1) |
| 动态环境 | 启用 `enable_visibility_cleanup: true` |
| 静态环境 | 禁用可见性清理，降低 CPU 占用 |

### 7.2 常见问题

| 问题 | 解决方案 |
|------|----------|
| TF 超时 | 增加 `time_tolerance` |
| 地图不更新 | 检查 TF 树完整性 |
| 点云稀疏 | 减小 `resolution` 或禁用体素滤波 |
| CPU 占用高 | 增大 `resolution`，禁用后处理 |

---

## 八、参考资料

1. [elevation_mapping GitHub](https://github.com/ANYbotics/elevation_mapping)
2. [grid_map 文档](https://github.com/ANYbotics/grid_map)
3. Fankhauser P., Bloesch M., Hutter M., "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization," IEEE Robotics and Automation Letters, 2018.
