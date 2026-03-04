cmake_minimum_required(VERSION 3.8)
project(elevation_mapping_livox_lidar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(elevation_mapping REQUIRED)
find_package(grid_map_rviz_plugin REQUIRED)

#############
## Install ##
#############

# Install directories
install(DIRECTORY
  config
  docs
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/tf_to_pose_publisher.py
  scripts/voxel_grid_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

##############
##   Test   ##
##############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()