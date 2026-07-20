cmake_minimum_required(VERSION 3.8)
project(rover_ekf_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

add_executable(rover_ekf_bridge src/main.cpp)
ament_target_dependencies(rover_ekf_bridge rclcpp nav_msgs)
target_link_libraries(rover_ekf_bridge px4_ros2_cpp::px4_ros2_cpp Eigen3::Eigen)

install(TARGETS rover_ekf_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
