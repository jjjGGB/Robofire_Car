/**
 * @file elevation_to_costmap_node.cpp
 * @brief 将 elevation_mapping 的 GridMap 转换为 Nav2 可用的 OccupancyGrid
 *
 * 数据流:
 *   elevation_mapping → /elevation_map (grid_map_msgs::GridMap)
 *                              ↓
 *              elevation_to_costmap_node (本节点: 平滑 + 坡度计算 + 通行性评估)
 *                              ↓
 *                    /elevation_costmap (nav_msgs::OccupancyGrid)
 *                              ↓
 *              Nav2 local_costmap → static_layer 订阅
 *
 * 处理流程:
 *   1. 对 elevation 层做 NaN-aware 均值平滑（仅平滑原始有效单元，不扩展 NaN 边界）
 *   2. 从平滑后的高程计算局部坡度（中心差分梯度）
 *   3. 坡度 → 通行性（含坡度死区，消除噪声和微小起伏的影响）:
 *        slope < min_slope → traversability = 1.0 (free)
 *        slope > max_slope → traversability = 0.0 (lethal)
 *        else             → 线性插值
 *   4. 使用 grid_map_ros 官方 converter 转换为 OccupancyGrid
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <cmath>

class ElevationToCostmapNode : public rclcpp::Node
{
public:
  ElevationToCostmapNode() : Node("elevation_to_costmap_node")
  {
    // ==================== 参数声明 ====================
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/elevation_map");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/elevation_costmap");

    // 坡度死区下限（弧度）: 低于此坡度视为平地，cost = 0
    // 默认 15° ≈ 0.2618 rad — 消除传感器噪声和微地形起伏的影响
    min_slope_rad_ = this->declare_parameter<double>("min_slope_rad", 0.2618);

    // 坡度上限（弧度）: 超过此坡度视为完全不可通行，cost = 100
    // 默认 30° ≈ 0.5236 rad
    max_slope_rad_ = this->declare_parameter<double>("max_slope_rad", 0.5236);

    // 平滑半径（单元格数），kernel size = 2*radius+1
    // radius=2 → 5x5 kernel
    smooth_radius_ = this->declare_parameter<int>("smooth_radius", 2);

    // 平滑核内最少有效单元数
    min_smooth_count_ = this->declare_parameter<int>("min_smooth_count", 3);

    // ==================== 订阅与发布 ====================
    grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ElevationToCostmapNode::gridMapCallback, this, std::placeholders::_1)
    );

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, pub_qos);

    RCLCPP_INFO(this->get_logger(),
      "ElevationToCostmap initialized.\n"
      "  Input: %s -> Output: %s\n"
      "  slope dead zone: [0, %.1f deg] = free\n"
      "  slope transition: [%.1f, %.1f deg] = gradual cost\n"
      "  slope lethal:     > %.1f deg = obstacle\n"
      "  smooth_radius: %d (kernel: %dx%d)",
      input_topic_.c_str(), output_topic_.c_str(),
      min_slope_rad_ * 180.0 / M_PI,
      min_slope_rad_ * 180.0 / M_PI, max_slope_rad_ * 180.0 / M_PI,
      max_slope_rad_ * 180.0 / M_PI,
      smooth_radius_, 2 * smooth_radius_ + 1, 2 * smooth_radius_ + 1);
  }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap gridMap;
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, gridMap)) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert GridMap message");
      return;
    }

    if (!gridMap.exists("elevation")) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "No 'elevation' layer in GridMap");
      return;
    }

    const auto& elevation = gridMap["elevation"];
    const int rows = gridMap.getSize()(0);
    const int cols = gridMap.getSize()(1);
    const float res = static_cast<float>(gridMap.getResolution());
    const float min_slope = static_cast<float>(min_slope_rad_);
    const float max_slope = static_cast<float>(max_slope_rad_);
    const float slope_range = max_slope - min_slope;

    // ==================== Step 1: NaN-aware 均值平滑 ====================
    // 关键：仅平滑原始有效单元，不将有效区域扩展到 NaN 边界
    // 避免在数据边界产生虚假梯度
    gridMap.add("elevation_smooth");
    auto& smoothed = gridMap["elevation_smooth"];

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        // 只平滑原始有数据的单元，不扩展到 NaN 区域
        if (std::isnan(elevation(r, c))) {
          smoothed(r, c) = NAN;
          continue;
        }

        float sum = 0.0f;
        int count = 0;
        for (int dr = -smooth_radius_; dr <= smooth_radius_; ++dr) {
          for (int dc = -smooth_radius_; dc <= smooth_radius_; ++dc) {
            const int nr = r + dr;
            const int nc = c + dc;
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
              const float val = elevation(nr, nc);
              if (!std::isnan(val)) {
                sum += val;
                count++;
              }
            }
          }
        }
        // 至少需要 min_smooth_count 个有效邻居，否则使用原始值
        smoothed(r, c) = (count >= min_smooth_count_) ? (sum / count) : elevation(r, c);
      }
    }

    // ==================== Step 2: 坡度 → 通行性 ====================
    // 使用中心差分法: slope = atan(sqrt(dz/dx^2 + dz/dy^2))
    // 坡度死区映射:
    //   slope < min_slope → trav = 1.0 (完全自由，消除噪声)
    //   slope > max_slope → trav = 0.0 (完全阻塞)
    //   中间区域          → 线性插值
    gridMap.add("traversability");
    auto& traversability = gridMap["traversability"];
    traversability.setConstant(NAN);

    for (int r = 1; r < rows - 1; ++r) {
      for (int c = 1; c < cols - 1; ++c) {
        const float h = smoothed(r, c);
        if (std::isnan(h)) continue;

        const float hL = smoothed(r, c - 1);
        const float hR = smoothed(r, c + 1);
        const float hU = smoothed(r - 1, c);
        const float hD = smoothed(r + 1, c);

        float dzdx = NAN, dzdy = NAN;

        if (!std::isnan(hL) && !std::isnan(hR)) {
          dzdx = (hR - hL) / (2.0f * res);
        } else if (!std::isnan(hL)) {
          dzdx = (h - hL) / res;
        } else if (!std::isnan(hR)) {
          dzdx = (hR - h) / res;
        }

        if (!std::isnan(hU) && !std::isnan(hD)) {
          dzdy = (hD - hU) / (2.0f * res);
        } else if (!std::isnan(hU)) {
          dzdy = (h - hU) / res;
        } else if (!std::isnan(hD)) {
          dzdy = (hD - h) / res;
        }

        if (!std::isnan(dzdx) || !std::isnan(dzdy)) {
          const float gx = std::isnan(dzdx) ? 0.0f : dzdx;
          const float gy = std::isnan(dzdy) ? 0.0f : dzdy;
          const float slope_val = std::atan(std::sqrt(gx * gx + gy * gy));

          // 坡度死区映射
          float trav;
          if (slope_val <= min_slope) {
            trav = 1.0f;   // 平地/缓坡：完全自由
          } else if (slope_val >= max_slope) {
            trav = 0.0f;   // 陡坡：完全阻塞
          } else {
            // 线性插值过渡区
            trav = 1.0f - (slope_val - min_slope) / slope_range;
          }

          traversability(r, c) = trav;
        }
      }
    }

    // ==================== Step 3: 通行性 → OccupancyGrid ====================
    // 使用 grid_map_ros 官方 converter：正确处理循环缓冲区和坐标映射
    // 映射: trav=1.0 → cost=0 (free), trav=0.0 → cost=100 (lethal), NaN → -1 (unknown)
    nav_msgs::msg::OccupancyGrid occupancyGrid;
    grid_map::GridMapRosConverter::toOccupancyGrid(
      gridMap, "traversability", 0.0f, 1.0f, occupancyGrid);

    // 覆盖时间戳为当前时间，确保 Nav2 TF 查询成功
    occupancyGrid.header.stamp = this->now();

    occupancy_grid_pub_->publish(occupancyGrid);
  }

  // ==================== 成员变量 ====================
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

  std::string input_topic_;
  std::string output_topic_;
  double min_slope_rad_;
  double max_slope_rad_;
  int smooth_radius_;
  int min_smooth_count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ElevationToCostmapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
