cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

# C++ 14 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

# ==================== 依赖包 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
# grid_map_costmap_2d 和 nav2_costmap_2d 不再需要
# elevation_to_costmap_node 直接从 elevation 层计算坡度

# ==================== 编译 elevation_to_costmap_node ====================
add_executable(elevation_to_costmap_node
  src/elevation_to_costmap_node.cpp
)
ament_target_dependencies(elevation_to_costmap_node
  rclcpp
  nav_msgs
  grid_map_msgs
  grid_map_core
  grid_map_ros
)

# ==================== 安装 ====================
install(TARGETS
  elevation_to_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config map PCD rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Python 脚本安装
install(PROGRAMS
  scripts/pointcloud_throttle.py
  scripts/amcl_nomotion_trigger.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==================== 测试 ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
