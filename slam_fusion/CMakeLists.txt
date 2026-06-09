cmake_minimum_required(VERSION 3.8)
project(slam_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 packages ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── PCL (add 'filters' for VoxelGrid — was missing in original) ────────────
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ── Executable ────────────────────────────────────────────────────────────
add_executable(semantic_fusion_node src/semantic_fusion.cpp)

# Fix for Conda environments where Conda's libcurl masks system libcurl
find_library(SYSTEM_CURL_LIB curl PATHS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
target_link_libraries(semantic_fusion_node ${PCL_LIBRARIES} ${SYSTEM_CURL_LIB})

ament_target_dependencies(semantic_fusion_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  message_filters
  pcl_conversions
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# ── Install ───────────────────────────────────────────────────────────────
install(TARGETS semantic_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tf_to_odom.py
  RENAME tf_to_odom
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_exporter.py
  scripts/semantic_evaluator_node.py
  scripts/performance_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()