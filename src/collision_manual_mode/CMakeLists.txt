cmake_minimum_required(VERSION 3.10)
project(collision_manual_mode)

# Require C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(collision_manual_node
  src/collision_manual_node.cpp
)

ament_target_dependencies(collision_manual_node
  rclcpp
  px4_msgs
  px4_ros2_cpp
  Eigen3
)

install(TARGETS collision_manual_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
